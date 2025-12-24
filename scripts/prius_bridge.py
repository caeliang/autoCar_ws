#!/usr/bin/env python3
"""
ROS2 to Gazebo Bridge for Prius Hybrid Vehicle Control
Subscribes to /prius_hybrid_123/cmd_vel and publishes keyboard commands to Gazebo

The PriusHybridPlugin responds to keyboard events, so we simulate them via Gazebo transport.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import threading
import time


class PriusBridge(Node):
    def __init__(self):
        super().__init__('prius_gazebo_bridge')
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/prius_hybrid_123/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_cmd_time = time.time()
        
        # Timer to send continuous commands
        self.timer = self.create_timer(0.05, self.send_command)
        
        self.get_logger().info('Prius Gazebo Bridge started!')
        self.get_logger().info('Listening on /prius_hybrid_123/cmd_vel')
        
    def cmd_vel_callback(self, msg):
        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z
        self.last_cmd_time = time.time()
        
    def send_command(self):
        # Auto-stop if no command received for 0.5 seconds
        if time.time() - self.last_cmd_time > 0.5:
            self.current_linear = 0.0
            self.current_angular = 0.0
        
        # Map linear/angular to throttle/brake/steering
        # Throttle: positive linear.x
        # Brake: negative linear.x or zero
        # Steering: angular.z
        
        throttle = max(0.0, min(1.0, self.current_linear / 50.0))  # Normalize to 0-1
        brake = max(0.0, min(1.0, -self.current_linear / 50.0)) if self.current_linear < 0 else 0.0
        steer = max(-1.0, min(1.0, self.current_angular))  # Already -1 to 1
        
        # Publish to Gazebo using gz topic
        try:
            # Throttle command
            if throttle > 0.01:
                subprocess.run([
                    'gz', 'topic', '-p', '/prius_hybrid_123/cmd',
                    '-m', 'gazebo.msgs.Pose',
                    f'position {{ x: {throttle} y: 0 z: 0 }} orientation {{ x: 0 y: 0 z: {steer} w: 1 }}'
                ], capture_output=True, timeout=0.1)
        except:
            pass


def main():
    rclpy.init()
    node = PriusBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
