#!/usr/bin/env python3
import argparse

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class OdomToTfBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("odom_to_sensor_tf_bridge")
        self.args = args
        self.broadcaster = TransformBroadcaster(self)
        self.latest_odom: Odometry | None = None
        self.latest_pose: PoseStamped | None = None
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        if args.source == "pose":
            self.create_subscription(PoseStamped, args.pose_topic, self.pose_callback, qos)
            self.get_logger().info(f"Publishing TF {args.parent_frame} -> {args.child_frame} from PoseStamped {args.pose_topic}")
        else:
            self.create_subscription(Odometry, args.odom_topic, self.odom_callback, qos)
            self.get_logger().info(f"Publishing TF {args.parent_frame} -> {args.child_frame} from Odometry {args.odom_topic}")
        if args.stamp_mode == "scan":
            self.create_subscription(LaserScan, args.scan_topic, self.scan_callback, qos)
            self.get_logger().info(f"TF stamps will be synchronized to LaserScan {args.scan_topic}")
        self.get_logger().info(f"TF stamp mode: {args.stamp_mode}")
        self.get_logger().warn(
            "Make sure no other node publishes the same child_frame_id. "
            f"Duplicate child frame '{args.child_frame}' can cause TF_OLD_DATA or disconnected trees."
        )

    def stamp_for(self, msg_stamp):
        if self.args.stamp_mode == "message":
            return msg_stamp
        return self.get_clock().now().to_msg()

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg
        if self.args.stamp_mode == "scan":
            return
        self.publish_from_odom(msg, self.stamp_for(msg.header.stamp))

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg
        if self.args.stamp_mode == "scan":
            return
        self.publish_from_pose(msg, self.stamp_for(msg.header.stamp))

    def scan_callback(self, msg: LaserScan) -> None:
        if self.args.source == "pose":
            if self.latest_pose is None:
                return
            self.publish_from_pose(self.latest_pose, msg.header.stamp)
        else:
            if self.latest_odom is None:
                return
            self.publish_from_odom(self.latest_odom, msg.header.stamp)

    def publish_from_odom(self, msg: Odometry, stamp) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.args.parent_frame
        transform.child_frame_id = self.args.child_frame
        transform.transform.translation.x = msg.pose.pose.position.x + self.args.x_offset
        transform.transform.translation.y = msg.pose.pose.position.y + self.args.y_offset
        transform.transform.translation.z = msg.pose.pose.position.z + self.args.z_offset
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)

    def publish_from_pose(self, msg: PoseStamped, stamp) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.args.parent_frame
        transform.child_frame_id = self.args.child_frame
        transform.transform.translation.x = msg.pose.position.x + self.args.x_offset
        transform.transform.translation.y = msg.pose.position.y + self.args.y_offset
        transform.transform.translation.z = msg.pose.position.z + self.args.z_offset
        transform.transform.rotation = msg.pose.orientation
        self.broadcaster.sendTransform(transform)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish odom -> sensor_link TF from /prius/odom or /localization/pose")
    parser.add_argument("--source", choices=["odom", "pose"], default="odom")
    parser.add_argument("--odom-topic", default="/prius/odom")
    parser.add_argument("--pose-topic", default="/localization/pose")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--parent-frame", default="odom")
    parser.add_argument("--child-frame", default="sensor_link")
    parser.add_argument("--x-offset", type=float, default=0.0)
    parser.add_argument("--y-offset", type=float, default=0.0)
    parser.add_argument("--z-offset", type=float, default=0.0)
    parser.add_argument("--stamp-mode", choices=["now", "message", "scan"], default="scan")
    parser.add_argument("--use-sim-time", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = OdomToTfBridge(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
