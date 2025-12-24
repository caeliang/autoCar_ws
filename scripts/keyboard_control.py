#!/usr/bin/env python3
"""
Prius Araba Kontrolu (ROS 2 + Gazebo Planar Move)
Topic: /prius/cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


def get_key(timeout=0.05):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class CarController(Node):
    def __init__(self):
        super().__init__('prius_controller')
        self.pub = self.create_publisher(Twist, '/prius/cmd_vel', 10)
        
        self.max_linear = 2.0
        self.max_angular = 1.0
        
        self.linear = 0.0
        self.angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.update)

    def update(self):
        # Smooth interpolation
        diff = self.target_linear - self.linear
        if abs(diff) > 0.01:
            self.linear += 0.05 if diff > 0 else -0.08
            if diff > 0:
                self.linear = min(self.linear, self.target_linear)
            else:
                self.linear = max(self.linear, self.target_linear)
        
        diff = self.target_angular - self.angular
        if abs(diff) > 0.01:
            self.angular += 0.1 if diff > 0 else -0.1
            if diff > 0:
                self.angular = min(self.angular, self.target_angular)
            else:
                self.angular = max(self.angular, self.target_angular)
        
        msg = Twist()
        # W/S icin linear.y (ters isaret cunku araba yonu)
        msg.linear.y = -self.linear
        # A/D icin angular.z (normal)
        msg.angular.z = self.angular
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CarController()

    print('\n' + '='*50)
    print('       PRIUS ARABA KONTROLU')
    print('='*50)
    print('  W = Ileri')
    print('  S = Geri')
    print('  A = Sola don')
    print('  D = Saga don')
    print('  X = Ani fren')
    print('  Q/E = Hiz ayarla')
    print('='*50)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            key = get_key(0.05)
            
            if key == 'w':
                node.target_linear = node.max_linear
            elif key == 's':
                node.target_linear = -node.max_linear * 0.5
            elif key == 'a':
                node.target_angular = node.max_angular
            elif key == 'd':
                node.target_angular = -node.max_angular
            elif key == 'x':
                node.linear = 0.0
                node.angular = 0.0
                node.target_linear = 0.0
                node.target_angular = 0.0
            elif key == 'q':
                node.max_linear = min(node.max_linear + 0.5, 5.0)
                print(f'\rHiz: {node.max_linear} m/s')
            elif key == 'e':
                node.max_linear = max(node.max_linear - 0.5, 0.5)
                print(f'\rHiz: {node.max_linear} m/s')
            elif key == '\x03':
                raise KeyboardInterrupt
            elif key == '':
                node.target_linear = 0.0
                node.target_angular = 0.0
            
            speed = abs(node.linear) * 3.6
            print(f'\rHiz: {speed:4.1f} km/h | Donus: {node.angular:+.2f}  ', end='', flush=True)

    except KeyboardInterrupt:
        print('\n\nCikis...')
    finally:
        msg = Twist()
        node.pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
