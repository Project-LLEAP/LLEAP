#!/usr/bin/env python3
"""
virtual_joy.py with ESTOP
  • /joy   : sensor_msgs/Joy  (axes[0], axes[1], buttons[0])
  • /estop : std_msgs/Bool    (True when Space or STOP clicked)
"""
import math, sys, pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

RATE_HZ   = 50
W, H      = 240, 300          # extra height for STOP bar
STICK_R   = 90
DEAD_ZONE = 0.05

class VirtualJoy(Node):
    def __init__(self):
        super().__init__('virtual_joy')
        self.pub_joy   = self.create_publisher(Joy,  '/joy',   10)
        self.pub_stop  = self.create_publisher(Bool, '/estop', 10)

        self.msg = Joy()
        self.msg.axes    = [0.0, 0.0]
        self.msg.buttons = [0]

        self.create_timer(1.0 / RATE_HZ, self.timer_cb)

    def timer_cb(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_joy.publish(self.msg)

    # ---- helper to blast a one-shot estop ----
    def send_estop(self):
        b = Bool();  b.data = True
        self.pub_stop.publish(b)
        self.get_logger().warn("EMERGENCY STOP SENT!")

def main():
    pygame.init()
    surf = pygame.display.set_mode((W, H))
    pygame.display.set_caption('Virtual Joy + E-STOP')
    clock = pygame.time.Clock()

    rclpy.init()
    joy = VirtualJoy()

    center = (W // 2, STICK_R + 30)

    dragging = False
    while rclpy.ok():
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                rclpy.shutdown();  pygame.quit();  sys.exit()

            # ---------- mouse ----------
            elif e.type == pygame.MOUSEBUTTONDOWN:
                x, y = e.pos
                if y > H - 40:                      # STOP bar
                    joy.send_estop()
                elif e.button == 1:                 # left drag
                    dragging = True
                elif e.button == 3:                 # right click toggles button0
                    joy.msg.buttons[0] ^= 1

            elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                dragging = False
                joy.msg.axes = [0.0, 0.0]

            # ---------- keyboard ----------
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    rclpy.shutdown(); pygame.quit(); sys.exit()
                if e.key == pygame.K_SPACE:
                    joy.send_estop()

        if dragging:
            mx, my = pygame.mouse.get_pos()
            dx = (mx - center[0]) / STICK_R
            dy = (my - center[1]) / STICK_R
            mag = math.hypot(dx, dy)
            if mag > 1.0:
                dx /= mag;  dy /= mag
            joy.msg.axes[0] = 0.0 if abs(dx) < DEAD_ZONE else dx
            joy.msg.axes[1] = 0.0 if abs(dy) < DEAD_ZONE else -dy

        # ------------ draw -------------
        surf.fill((30, 30, 30))
        # stick outline & knob
        pygame.draw.circle(surf, (60,180,240), center, STICK_R, 3)
        knob = (int(center[0] + joy.msg.axes[0]*STICK_R),
                int(center[1] - joy.msg.axes[1]*STICK_R))
        pygame.draw.circle(surf, (240,80,80), knob, 18)
        # STOP bar
        pygame.draw.rect(surf, (180,0,0), (0, H-40, W, 40))
        pygame.draw.rect(surf, (255,60,60), (0, H-40, W, 40), 3)
        font = pygame.font.SysFont(None, 46)
        stop_text = font.render('E-STOP SPACE', True, (255, 255, 255))
        text_rect = stop_text.get_rect(center=(W//2, H-20))
        surf.blit(stop_text, text_rect)

        pygame.display.flip()

        rclpy.spin_once(joy, timeout_sec=0.0)
        clock.tick(RATE_HZ)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
