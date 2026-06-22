#!/usr/bin/env python3
import csv
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler

CSV_PATH = "/home/ranim/autoCar_ws/waypoints/planned_route.csv"

class RoutePublisher(Node):
    def __init__(self):
        super().__init__("route_csv_path_publisher")
        self.declare_parameter("csv_path", CSV_PATH)
        self.csv_path = self.get_parameter("csv_path").get_parameter_value().string_value

        self.pub = self.create_publisher(Path, "/waypoints/path", 10)
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, "/waypoints/status", status_qos)

        self.path = self.load_path(self.csv_path)
        self.timer = self.create_timer(0.5, self.publish_path)
        self.get_logger().info(f"Loaded {len(self.path.poses)} route points from {self.csv_path}")

    def load_path(self, path):
        msg = Path()
        msg.header.frame_id = "map"

        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        for i, row in enumerate(rows):
            x = float(row["x"])
            y = float(row["y"])

            if "yaw" in row and row["yaw"] != "":
                yaw = math.radians(float(row["yaw"]))
            elif i < len(rows) - 1:
                nx = float(rows[i + 1]["x"])
                ny = float(rows[i + 1]["y"])
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0

            q = quaternion_from_euler(0.0, 0.0, yaw)

            p = PoseStamped()
            lane_id = row.get("lane_id", "")
            direction_group = row.get("direction_group", "")
            p.header.frame_id = f"map|{lane_id}|{direction_group}"
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.5
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            msg.poses.append(p)

        return msg

    def publish_path(self):
        now = self.get_clock().now().to_msg()
        self.path.header.stamp = now
        for p in self.path.poses:
            p.header.stamp = now
        self.pub.publish(self.path)

        status = String()
        status.data = "ACTIVE"
        self.status_pub.publish(status)

def main():
    rclpy.init()
    node = RoutePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
