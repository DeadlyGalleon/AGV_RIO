#!/usr/bin/env python3
import rospy, pygame
from std_msgs.msg import Float64

if __name__ == "__main__":
    rospy.init_node("dual_wheel_teleop")
    pub_l = rospy.Publisher("/agv/left_wheel_velocity_controller/command",  Float64, queue_size=10)
    pub_r = rospy.Publisher("/agv/right_wheel_velocity_controller/command", Float64, queue_size=10)

    rate = rospy.Rate(50)
    pygame.init()
    try:
        screen = pygame.display.set_mode((360,120))
    except pygame.error:
        screen = None

    wl = 0.0; wr = 0.0
    step = 1     # bước tăng rad/s mỗi lần nhấn
    maxw = 30.0    # giới hạn tốc độ bánh

    def clamp(x, a, b): return max(a, min(b, x))

    while not rospy.is_shutdown():
        for e in pygame.event.get():
            if e.type == pygame.QUIT: rospy.signal_shutdown("quit")
            if e.type == pygame.KEYDOWN and e.key in (pygame.K_ESCAPE, pygame.K_q):
                rospy.signal_shutdown("quit")

        k = pygame.key.get_pressed()

        # W/S: tăng/giảm cả 2 bánh (tiến/lùi)
        if k[pygame.K_w]: wl += step; wr += step
        if k[pygame.K_s]: wl -= step; wr -= step

        # A/D: quay – trái nhanh/phải chậm và ngược lại
        if k[pygame.K_a]: wl += step; wr -= step
        if k[pygame.K_d]: wl -= step; wr += step

        # SPACE: dừng
        if k[pygame.K_SPACE]: wl = 0.0; wr = 0.0

        wl = clamp(wl, -maxw, maxw)
        wr = clamp(wr, -maxw, maxw)

        pub_l.publish(Float64(wl))
        pub_r.publish(Float64(wr))
        rate.sleep()
