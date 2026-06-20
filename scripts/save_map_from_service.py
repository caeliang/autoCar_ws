#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
from PIL import Image
import numpy as np
import yaml
import os

OUT = "/home/ranim/autoCar_ws/maps/new_occupancy_map"

class SaveMapService(Node):
    def __init__(self):
        super().__init__("save_map_from_service")
        self.client = self.create_client(GetMap, "/slam_toolbox/dynamic_map")

    def run(self):
        self.get_logger().info("Waiting for /slam_toolbox/dynamic_map...")
        self.client.wait_for_service()

        future = self.client.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)

        msg = future.result().map
        w, h = msg.info.width, msg.info.height
        print(f"Map size: width={w}, height={h}, data_len={len(msg.data)}")

        if w == 0 or h == 0 or len(msg.data) == 0:
            raise RuntimeError("Dynamic map is empty. SLAM Toolbox has not produced a valid map yet.")

        data = np.array(msg.data, dtype=np.int16).reshape((h, w))
        img = np.zeros((h, w), dtype=np.uint8)

        img[data == -1] = 205
        img[data == 0] = 254
        img[data > 50] = 0
        img = np.flipud(img)

        pgm_path = OUT + ".pgm"
        yaml_path = OUT + ".yaml"

        Image.fromarray(img).save(pgm_path)

        meta = {
            "image": os.path.basename(pgm_path),
            "mode": "trinary",
            "resolution": float(msg.info.resolution),
            "origin": [
                float(msg.info.origin.position.x),
                float(msg.info.origin.position.y),
                0.0
            ],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25
        }

        with open(yaml_path, "w") as f:
            yaml.dump(meta, f, sort_keys=False)

        self.get_logger().info(f"Saved {pgm_path}")
        self.get_logger().info(f"Saved {yaml_path}")

rclpy.init()
node = SaveMapService()
node.run()
rclpy.shutdown()
