#!/usr/bin/env python3
import os
import sys
import math
import time
import pygame
import rospy
from geometry_msgs.msg import Twist

HELP = """
W/S: tăng/giảm tốc tiến-lùi
A/D: tăng/giảm quay tại chỗ (yaw)
SPACE: phanh khẩn cấp (về 0)
X: reset mềm (về 0)
Q hoặc ESC: thoát
"""

def clamp(v, vmin, vmax):
    return max(vmin, min(v, vmax))

def main():
    # Cho phép chạy cả khi không có DISPLAY (nếu cần):
    if "DISPLAY" not in os.environ:
        os.environ["SDL_VIDEODRIVER"] = "dummy"

    rospy.init_node("pygame_wasd_teleop")
    cmd_topic = rospy.get_param("~cmd_topic", "/agv/diff_drive/cmd_vel")

    pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
    rate_hz = rospy.get_param("~rate_hz", 50)
    rate = rospy.Rate(rate_hz)

    # Giới hạn & bước tăng
    max_lin = rospy.get_param("~max_linear", 20.0)     # m/s
    max_ang = rospy.get_param("~max_angular", 30.0)    # rad/s
    step_lin = rospy.get_param("~step_linear", 2.0)   # m/s mỗi lần nhấn
    step_ang = rospy.get_param("~step_angular", -5.0)  # rad/s mỗi lần nhấn

    # Hệ số giảm dần nếu không bấm phím (ramp về 0)
    decay_lin = rospy.get_param("~decay_linear", 0.9)   # 0..1
    decay_ang = rospy.get_param("~decay_angular", 0.85) # 0..1

    vx = 0.0     # linear.x hiện tại
    wz = 0.0     # angular.z hiện tại

    pygame.init()
    # Tạo cửa sổ nhỏ để Pygame bắt keyboard focus (nếu có GUI)
    try:
        screen = pygame.display.set_mode((420, 180))
        pygame.display.set_caption("WASD Teleop (Pygame) – "+cmd_topic)
    except pygame.error:
        screen = None  # headless

    font = pygame.font.SysFont(None, 20) if screen else None
    last_draw = 0.0

    rospy.loginfo("Pygame teleop started. Topic: %s", cmd_topic)
    print(HELP)

    try:
        while not rospy.is_shutdown():
            # Đọc event
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown("Pygame window closed.")
                    return
                if event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        rospy.signal_shutdown("User quit.")
                        return
                    elif event.key == pygame.K_SPACE:
                        vx = 0.0
                        wz = 0.0
                    elif event.key == pygame.K_x:
                        vx = 0.0
                        wz = 0.0

            # Đọc trạng thái phím giữ
            keys = pygame.key.get_pressed()
            if keys[pygame.K_w]:
                vx += step_lin
            if keys[pygame.K_s]:
                vx -= step_lin
            if keys[pygame.K_a]:
                wz += step_ang
            if keys[pygame.K_d]:
                wz -= step_ang

            # Giới hạn
            vx = clamp(vx, -max_lin, max_lin)
            wz = clamp(wz, -max_ang, max_ang)

            # Nếu không giữ phím tăng/giảm, áp dụng decay về 0 cho mượt
            if not (keys[pygame.K_w] or keys[pygame.K_s]):
                vx *= decay_lin
                if abs(vx) < 1e-3: vx = 0.0
            if not (keys[pygame.K_a] or keys[pygame.K_d]):
                wz *= decay_ang
                if abs(wz) < 1e-3: wz = 0.0

            # Publish
            msg = Twist()
            msg.linear.x  = vx
            msg.angular.z = wz
            pub.publish(msg)

            # Vẽ UI đơn giản
            if screen and time.time() - last_draw > 0.05:
                screen.fill((30, 30, 30))
                lines = [
                    f"Topic: {cmd_topic}",
                    f"linear.x = {vx:+.2f} m/s   (max {max_lin})",
                    f"angular.z = {wz:+.2f} rad/s (max {max_ang})",
                    "Keys: W/S speed, A/D turn, SPACE brake, X reset, Q/ESC quit",
                ]
                y = 20
                for ln in lines:
                    img = font.render(ln, True, (220, 220, 220))
                    screen.blit(img, (15, y))
                    y += 24
                pygame.display.flip()
                last_draw = time.time()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        # Ngừng xe khi thoát
        stop = Twist()
        pub.publish(stop)
        pygame.quit()
        rospy.loginfo("Pygame teleop stopped; robot commanded to stop.")

if __name__ == "__main__":
    main()
