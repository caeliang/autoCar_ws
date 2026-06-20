#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import yaml
import os
import numpy as np

class MapSaver(Node):
    def __init__(self):
        super().__init__('save_map_once')
        self.out = '/home/ranim/autoCar_ws/maps/new_occupancy_map'
        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.cb, qos)
        self.get_logger().info('Waiting for /map...')

    def cb(self, msg):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape((h, w))

        img = np.zeros((h, w), dtype=np.uint8)
        img[data == -1] = 205
        img[data == 0] = 254
        img[data > 50] = 0
        img = np.flipud(img)

        pgm = self.out + '.pgm'
        yaml_path = self.out + '.yaml'

        Image.fromarray(img).save(pgm)

        meta = {
            'image': os.path.basename(pgm),
            'mode': 'trinary',
            'resolution': float(msg.info.resolution),
            'origin': [
                float(msg.info.origin.position.x),
                float(msg.info.origin.position.y),
                0.0
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }

        with open(yaml_path, 'w') as f:
            yaml.dump(meta, f, sort_keys=False)

        self.get_logger().info(f'Saved: {pgm}')
        self.get_logger().info(f'Saved: {yaml_path}')
        rclpy.shutdown()

rclpy.init()
node = MapSaver()
rclpy.spin(node)
