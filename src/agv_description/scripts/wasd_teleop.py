#!/usr/bin/env python3
import os
import rospy
from geometry_msgs.msg import Twist

# Pygame chỉ tạo cửa sổ nhỏ, có thể ẩn hoặc dùng SDL_VIDEODRIVER=dummy khi headless
import pygame

KEY_BINDINGS = {
    pygame.K_w: ("lin", +1.0),  # tiến
    pygame.K_s: ("lin", -1.0),  # lùi
    pygame.K_a: ("ang", +1.0),  # quay trái (CCW)
    pygame.K_d: ("ang", -1.0),  # quay phải (CW)
}

HELP_TEXT = """
W/S: tiến / lùi
A/D: quay trái / quay phải
Shift giữ: tăng tốc tạm thời (x2)
 +/- : tăng/giảm tốc độ tối đa
 Space: dừng khẩn (twist=0)
 Esc hoặc Q: thoát
"""

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def main():
    rospy.init_node("wasd_teleop")

    # Cho phép remap qua arg/param
    topic = rospy.get_param("~cmd_vel_topic", "/agv_car/diff_drive_controller/cmd_vel")
    hz = rospy.get_param("~rate", 50)
    max_lin = rospy.get_param("~max_linear", 0.8)   # m/s
    max_ang = rospy.get_param("~max_angular", 2.0)  # rad/s
    lin_step = rospy.get_param("~lin_step", 0.02)
    ang_step = rospy.get_param("~ang_step", 0.06)

    pub = rospy.Publisher(topic, Twist, queue_size=10)
    rate = rospy.Rate(hz)

    # Khởi tạo pygame
    if "SDL_VIDEODRIVER" not in os.environ:
        # Tạo window bé để nhận keyboard focus
        pygame.display.init()
        screen = pygame.display.set_mode((320, 80))
        pygame.display.set_caption("WASD Teleop (focus cửa sổ này)")
    else:
        pygame.display.init()  # dummy driver vẫn ok

    pygame.font.init()
    font = pygame.font.SysFont(None, 18)
    clock = pygame.time.Clock()

    # Trạng thái tốc độ tức thời (throttle) và giới hạn
    cur_lin = 0.0
    cur_ang = 0.0
    lin_limit = max_lin
    ang_limit = max_ang

    running = True
    print(HELP_TEXT)

    while not rospy.is_shutdown() and running:
        # Xử lý event
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_SPACE:
                    cur_lin = 0.0
                    cur_ang = 0.0
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):  # '+' trên nhiều bàn phím là '=' + Shift
                    lin_limit = clamp(lin_limit + 0.1, 0.1, 3.0)
                    ang_limit = clamp(ang_limit + 0.2, 0.2, 6.0)
                    print(f"[+] max_linear={lin_limit:.2f} m/s, max_angular={ang_limit:.2f} rad/s")
                elif event.key == pygame.K_MINUS:
                    lin_limit = clamp(lin_limit - 0.1, 0.1, 3.0)
                    ang_limit = clamp(ang_limit - 0.2, 0.2, 6.0)
                    print(f"[-] max_linear={lin_limit:.2f} m/s, max_angular={ang_limit:.2f} rad/s")

        # Trạng thái phím giữ
        keys = pygame.key.get_pressed()

        # Hệ số turbo khi giữ Shift
        turbo = 2.0 if (keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]) else 1.0

        # Tích lũy mượt (kiểu ramp) cho linear
        if keys[pygame.K_w]:
            cur_lin = clamp(cur_lin + lin_step, -lin_limit * turbo, lin_limit * turbo)
        elif keys[pygame.K_s]:
            cur_lin = clamp(cur_lin - lin_step, -lin_limit * turbo, lin_limit * turbo)
        else:
            # tự giảm về 0 cho mượt
            if abs(cur_lin) < lin_step:
                cur_lin = 0.0
            else:
                cur_lin -= lin_step * (1 if cur_lin > 0 else -1)

        # Tích lũy mượt cho angular
        if keys[pygame.K_a]:
            cur_ang = clamp(cur_ang + ang_step, -ang_limit * turbo, ang_limit * turbo)
        elif keys[pygame.K_d]:
            cur_ang = clamp(cur_ang - ang_step, -ang_limit * turbo, ang_limit * turbo)
        else:
            if abs(cur_ang) < ang_step:
                cur_ang = 0.0
            else:
                cur_ang -= ang_step * (1 if cur_ang > 0 else -1)

        # Xuất Twist
        twist = Twist()
        twist.linear.x = cur_lin
        twist.angular.z = cur_ang
        pub.publish(twist)

        # Vẽ text nhỏ (nếu có cửa sổ)
        if pygame.display.get_init() and pygame.display.get_surface():
            screen.fill((30, 30, 30))
            lines = [
                "WASD Teleop",
                f"topic: {topic}",
                f"lin: {cur_lin:+.2f} m/s  (max {lin_limit:.2f})",
                f"ang: {cur_ang:+.2f} rad/s (max {ang_limit:.2f})",
                "Shift=turbo, Space=stop, +/-=adjust max",
                "Esc/Q=quit",
            ]
            y = 5
            for l in lines:
                img = font.render(l, True, (220, 220, 220))
                screen.blit(img, (8, y))
                y += 16
            pygame.display.flip()

        clock.tick(hz)
        rate.sleep()

    # gửi 0 lần cuối
    pub.publish(Twist())
    pygame.quit()

if __name__ == "__main__":
    main()
