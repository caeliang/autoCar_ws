#!/usr/bin/env python3
"""
Keyboard teleop for Prius in Gazebo (ROS 2)
Publishes to: /prius_hybrid_123/cmd_vel

Controls:
  w/s : forward/backward
  a/d : rotate left/right
  q   : increase linear speed
  e   : decrease linear speed
  z   : increase angular speed
  x   : decrease angular speed
  space : stop
  Ctrl-C: quit

This is a lightweight local teleop script (no external deps beyond ROS2 and Python stdlib).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


def get_key(timeout=0.1):
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


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('prius_keyboard_teleop')
        self.pub = self.create_publisher(Twist, '/prius_hybrid_123/cmd_vel', 10)
        self.linear_speed = 1.5  # m/s initial
        self.angular_speed = 1.0  # rad/s initial
        self.get_logger().info('Keyboard teleop node started. Focus this terminal and use keys to control the Prius.')

    def publish_twist(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)


def print_instructions(linear, angular):
    print('\nKeyboard teleop controls:')
    print('  w/s : forward/backward')
    print('  a/d : rotate left/right')
    print('  q/e : increase/decrease linear speed')
    print('  z/x : increase/decrease angular speed')
    print('  space : stop')
    print('  Ctrl-C : quit')
    print(f'Current speeds -> linear: {linear:.2f} m/s, angular: {angular:.2f} rad/s')


def main():
    rclpy.init()
    node = KeyboardTeleop()

    try:
        print_instructions(node.linear_speed, node.angular_speed)
        while rclpy.ok():
            key = get_key(0.1)

            if key == 'w':
                node.publish_twist(node.linear_speed, 0.0)
                node.get_logger().debug('forward')
            elif key == 's':
                node.publish_twist(-node.linear_speed, 0.0)
                node.get_logger().debug('back')
            elif key == 'a':
                node.publish_twist(0.0, node.angular_speed)
                node.get_logger().debug('turn left')
            elif key == 'd':
                node.publish_twist(0.0, -node.angular_speed)
                node.get_logger().debug('turn right')
            elif key == 'q':
                node.linear_speed *= 1.1
                print(f'Linear speed: {node.linear_speed:.2f} m/s')
            elif key == 'e':
                node.linear_speed *= 0.9
                print(f'Linear speed: {node.linear_speed:.2f} m/s')
            elif key == 'z':
                node.angular_speed *= 1.1
                print(f'Angular speed: {node.angular_speed:.2f} rad/s')
            elif key == 'x':
                node.angular_speed *= 0.9
                print(f'Angular speed: {node.angular_speed:.2f} rad/s')
            elif key == ' ':
                node.publish_twist(0.0, 0.0)
                node.get_logger().info('STOP')
            elif key == '\x03':  # Ctrl-C
                raise KeyboardInterrupt
            else:
                # when no key or unrecognized key, publish zero to avoid drifting
                if key == '':
                    # don't spam zero if nothing changed; small idle sleep handles it
                    pass
                else:
                    node.get_logger().debug(f'Unknown key: {repr(key)}')

    except KeyboardInterrupt:
        print('\nExiting keyboard teleop')
    finally:
        # ensure we publish zero velocity on exit
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
