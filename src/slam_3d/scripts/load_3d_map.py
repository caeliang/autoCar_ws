#!/usr/bin/env python3
"""
Load and Publish 3D Map
Loads a saved .bt octomap file and publishes it
"""

import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
from std_msgs.msg import Header
import sys
import os


class MapLoader3D(Node):
    def __init__(self, map_file):
        super().__init__('map_loader_3d')
        self.map_file = map_file
        
        # Create publisher
        self.publisher = self.create_publisher(Octomap, '/octomap_binary', 10)
        
        # Load and publish map
        self.load_and_publish()
    
    def load_and_publish(self):
        """Load octomap from file and publish it"""
        if not os.path.exists(self.map_file):
            self.get_logger().error(f'Map file not found: {self.map_file}')
            return False
        
        try:
            with open(self.map_file, 'rb') as f:
                map_data = f.read()
            
            # Create Octomap message
            msg = Octomap()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.binary = True
            msg.id = 'OcTree'
            msg.resolution = 0.1  # Default, will be overridden by actual map
            msg.data = list(map_data)
            
            # Publish
            self.get_logger().info(f'Publishing map: {self.map_file}')
            self.get_logger().info(f'Map size: {len(map_data)} bytes')
            
            # Publish periodically
            timer = self.create_timer(1.0, lambda: self.publisher.publish(msg))
            
            self.get_logger().info('Map published! Press Ctrl+C to stop.')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {str(e)}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print('\nUsage: ros2 run slam_3d load_3d_map.py <map_file.bt>')
        print('Example: ros2 run slam_3d load_3d_map.py /home/ranim/autoCar_ws/maps/my_3d_map.bt\n')
        return
    
    map_file = sys.argv[1]
    
    print(f'\n╔══════════════════════════════════════╗')
    print(f'║   3D Map Loader (Octomap)           ║')
    print(f'╚══════════════════════════════════════╝\n')
    print(f'Loading: {map_file}\n')
    
    loader = MapLoader3D(map_file)
    
    try:
        rclpy.spin(loader)
    except KeyboardInterrupt:
        print('\nShutting down...')
    
    loader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
