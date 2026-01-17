#!/usr/bin/env python3
"""
Map Saver for SLAM Toolbox
Handles QoS compatibility with slam_toolbox
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image
import yaml
import sys
import os


class MapSaver(Node):
    def __init__(self, map_name):
        super().__init__('map_saver_custom')
        
        self.base_map_name = map_name
        self.map_name = self._get_next_map_name(map_name)
        self.map_received = False
        
        # QoS profile matching slam_toolbox publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile
        )
        
        self.get_logger().info(f'Waiting for map on /map topic...')
    
    def _get_next_map_name(self, base_name):
        """Find next available map name with numbering"""
        counter = 1
        while True:
            map_pgm = f"{base_name}_{counter:03d}.pgm"
            map_yaml = f"{base_name}_{counter:03d}.yaml"
            if not os.path.exists(map_pgm) and not os.path.exists(map_yaml):
                return f"{base_name}_{counter:03d}"
            counter += 1
    
    def map_callback(self, msg):
        if self.map_received:
            return
            
        self.map_received = True
        self.get_logger().info(f'Map received: {msg.info.width}x{msg.info.height}')
        
        # Save the map
        self.save_map(msg)
        
        # Shutdown after saving
        rclpy.shutdown()
    
    def save_map(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin
        
        # Convert occupancy grid to image
        # OccupancyGrid: -1 = unknown, 0 = free, 100 = occupied
        # PGM: 205 = unknown, 254 = free, 0 = occupied
        
        img_data = np.zeros((height, width), dtype=np.uint8)
        
        for i, value in enumerate(msg.data):
            row = i // width
            col = i % width
            
            if value == -1:  # Unknown
                img_data[height - 1 - row][col] = 205
            elif value == 0:  # Free
                img_data[height - 1 - row][col] = 254
            elif value >= 50:  # Occupied
                img_data[height - 1 - row][col] = 0
            else:  # Low probability occupied
                img_data[height - 1 - row][col] = int(254 - (value * 254 / 100))
        
        # Save PGM image
        img = Image.fromarray(img_data, mode='L')
        pgm_path = f'{self.map_name}.pgm'
        img.save(pgm_path)
        self.get_logger().info(f'✓ Saved: {pgm_path}')
        
        # Save YAML metadata
        yaml_data = {
            'image': os.path.basename(pgm_path),
            'resolution': resolution,
            'origin': [origin.position.x, origin.position.y, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }
        
        yaml_path = f'{self.map_name}.yaml'
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
        
        self.get_logger().info(f'✓ Saved: {yaml_path}')
        self.get_logger().info(f'\n✓ Harita başarıyla kaydedildi!')
        self.get_logger().info(f'  Map: {self.map_name}')
        self.get_logger().info(f'  Size: {width}x{height} pixels')
        self.get_logger().info(f'  Resolution: {resolution} m/pixel')


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 save_map.py <map_name>')
        print('Example: python3 save_map.py /home/ranim/autoCar_ws/maps/city_map')
        sys.exit(1)
    
    map_name = sys.argv[1]
    
    rclpy.init()
    node = MapSaver(map_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
