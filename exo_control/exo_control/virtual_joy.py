#!/usr/bin/env python3
"""
  • /joy     : sensor_msgs/Joy   (axes[0], axes[1], buttons[0])
  • /enable  : std_msgs/Bool     (True = enabled / assist on)
  • /estop   : std_msgs/Bool     (True = EMERGENCY STOP latched)
"""

import math, sys, pygame, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg   import Bool

# UI constants
RATE_HZ   = 50
W, H      = 260, 360          # extra height for two bars
STICK_R   = 90
DEAD_ZONE = 0.05
BAR_H     = 44
ENABLE_Y  = H - BAR_H * 2
ESTOP_Y   = H - BAR_H

class JoyGUI(Node):
    def __init__(self):
        super().__init__('virtual_joy_gui')

        self.pub_joy   = self.create_publisher(Joy,  '/joy',   10)
        self.pub_en    = self.create_publisher(Bool, '/enable', 10)
        self.pub_stop  = self.create_publisher(Bool, '/estop', 10)
        self.sub_en    = self.create_subscription(Bool, '/enable', self.en_cb,   10)
        self.sub_stop  = self.create_subscription(Bool, '/estop',  self.stop_cb, 10)

        self.enabled = False
        self.estop   = False     # latched stop
        self.joy_msg = Joy()
        self.joy_msg.axes, self.joy_msg.buttons = [0.0, 0.0], [0]

        self.create_timer(1.0 / RATE_HZ, self.timer_cb)

    # ROS I/O
    def timer_cb(self):
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_joy.publish(self.joy_msg)

    def en_cb(self, msg: Bool):
        self.enabled = bool(msg.data)

    def stop_cb(self, msg: Bool):
        self.estop = bool(msg.data)
        if self.estop:
            self.enabled = False   # force disable

    def send_enable(self, val: bool):
        self.enabled = val
        m = Bool(); m.data = val
        self.pub_en.publish(m)

    def toggle_enable(self):
        if not self.estop:
            self.send_enable(not self.enabled)

    def latch_estop(self):
        if not self.estop:
            self.estop = True
            self.enabled = False
            m = Bool(); m.data = True
            self.pub_stop.publish(m)
            self.get_logger().warn("EMERGENCY STOP LATCHED")

    def clear_estop(self):
        if self.estop:
            self.estop = False
            m = Bool(); m.data = False
            self.pub_stop.publish(m)
            self.get_logger().warn("E-STOP cleared")


def main():
    pygame.init()
    surf  = pygame.display.set_mode((W, H))
    pygame.display.set_caption('Virtual Joy + Enable + E-STOP')
    font  = pygame.font.SysFont(None, 34)
    clock = pygame.time.Clock()

    rclpy.init()
    gui = JoyGUI()

    center = (W // 2, STICK_R + 40)
    dragging = False

    while rclpy.ok():
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                rclpy.shutdown(); pygame.quit(); sys.exit()

            elif e.type == pygame.MOUSEBUTTONDOWN:
                x, y = e.pos
                if y >= ESTOP_Y:                     # red bar area
                    if e.button == 1: gui.latch_estop()      # left click = ESTOP
                    elif e.button == 3: gui.clear_estop()    # right click = reset
                elif y >= ENABLE_Y:                  # green/grey bar
                    if e.button == 1: gui.toggle_enable()
                elif e.button == 1:
                    dragging = True
                elif e.button == 3:
                    gui.joy_msg.buttons[0] ^= 1

            elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                dragging = False
                gui.joy_msg.axes = [0.0, 0.0]

            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    rclpy.shutdown(); pygame.quit(); sys.exit()
                if e.key == pygame.K_SPACE:
                    gui.latch_estop()
                if e.key == pygame.K_r:
                    gui.clear_estop()
                if e.key == pygame.K_e:
                    gui.toggle_enable()

        # joystick drag
        if dragging:
            mx, my = pygame.mouse.get_pos()
            dx = (mx - center[0]) / STICK_R
            dy = (my - center[1]) / STICK_R
            mag = math.hypot(dx, dy)
            if mag > 1.0:
                dx /= mag; dy /= mag
            gui.joy_msg.axes[0] = 0.0 if abs(dx) < DEAD_ZONE else dx
            gui.joy_msg.axes[1] = 0.0 if abs(dy) < DEAD_ZONE else -dy

        # draw
        surf.fill((35, 35, 35))

        # stick
        pygame.draw.circle(surf, (60,180,240), center, STICK_R, 3)
        knob = (int(center[0] + gui.joy_msg.axes[0]*STICK_R),
                int(center[1] - gui.joy_msg.axes[1]*STICK_R))
        pygame.draw.circle(surf, (240,80,80), knob, 18)

        # enable bar
        en_color = (0,150,0) if gui.enabled else (70,70,70)
        pygame.draw.rect(surf, en_color, (0, ENABLE_Y, W, BAR_H))
        pygame.draw.rect(surf, (255,255,255), (0, ENABLE_Y, W, BAR_H), 2)
        en_text = font.render('ENABLE', True, (255,255,255))
        surf.blit(en_text, en_text.get_rect(center=(W//2, ENABLE_Y + BAR_H//2)))

        # estop bar
        stop_color = (180,0,0) if gui.estop else (120,0,0)
        pygame.draw.rect(surf, stop_color, (0, ESTOP_Y, W, BAR_H))
        pygame.draw.rect(surf, (255,255,255), (0, ESTOP_Y, W, BAR_H), 2)
        stop_txt = 'E-STOP (SPACE)'
        s_text = font.render(stop_txt, True, (255,255,255))
        surf.blit(s_text, s_text.get_rect(center=(W//2, ESTOP_Y + BAR_H//2)))

        pygame.display.flip()
        rclpy.spin_once(gui, timeout_sec=0.0)
        clock.tick(RATE_HZ)

if __name__ == '__main__':
    try: main()
    except KeyboardInterrupt:
        pass
