#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_CSV = "/home/ranim/autoCar_ws/waypoints/lane_points_left_outer_vertical_fixed_like_right.csv"

NO_ENTRY_CLASSES = {
    "girilmez",
    "no_entry",
    "no entry",
    "noentry",
    "do_not_enter",
}

RED_LIGHT_CLASSES = {
    "red",
    "kirmizi",
    "kırmızı",
    "red_light",
}

GREEN_LIGHT_CLASSES = {
    "green",
    "yesil",
    "yeşil",
    "green_light",
}

IGNORED_SEGMENT_PREFIXES = (
    "smooth_",
    "roundabout_lane_blend_arc_",
    "RING_",
    "reference_",
)


@dataclass
class SegmentCandidate:
    parent_segment_id: str
    center_x: float
    center_y: float
    direction_group: str


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clean_class_name(value: Any) -> str:
    return str(value).strip().lower()


class TrafficSignLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("traffic_sign_localizer_node")

        self.declare_parameter("detections_topic", "/perception/yolo/detections")
        self.declare_parameter("pose_topic", "/localization/pose")
        self.declare_parameter("event_topic", "/traffic_sign/event")
        self.declare_parameter("csv_path", DEFAULT_CSV)
        self.declare_parameter("min_confidence", 0.55)
        self.declare_parameter("camera_horizontal_fov_deg", 90.0)
        self.declare_parameter("max_segment_distance", 35.0)
        self.declare_parameter("angle_weight", 2.0)
        self.declare_parameter("distance_weight", 0.05)
        self.declare_parameter("cooldown_sec", 2.0)

        self.detections_topic = self.get_parameter("detections_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.event_topic = self.get_parameter("event_topic").value
        self.csv_path = self.get_parameter("csv_path").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.camera_horizontal_fov = math.radians(
            float(self.get_parameter("camera_horizontal_fov_deg").value)
        )
        self.max_segment_distance = float(self.get_parameter("max_segment_distance").value)
        self.angle_weight = float(self.get_parameter("angle_weight").value)
        self.distance_weight = float(self.get_parameter("distance_weight").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)

        self.segments = self.load_segments(self.csv_path)
        self.pose: Optional[PoseStamped] = None
        self.last_publish_time: Dict[str, float] = {}

        self.event_pub = self.create_publisher(String, self.event_topic, 10)
        self.detections_sub = self.create_subscription(
            String,
            self.detections_topic,
            self.on_detections,
            10,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.on_pose,
            10,
        )

        self.get_logger().info(
            f"TrafficSignLocalizer ready | segments={len(self.segments)} "
            f"csv={self.csv_path} detections={self.detections_topic} pose={self.pose_topic} "
            f"event={self.event_topic}"
        )

    def load_segments(self, csv_path: str) -> List[SegmentCandidate]:
        if not os.path.isfile(csv_path):
            raise RuntimeError(f"Waypoint CSV bulunamadi: {csv_path}")

        grouped: Dict[Tuple[str, str], Dict[str, Any]] = {}
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            fieldnames = reader.fieldnames or []
            missing = {"x", "y", "parent_segment_id", "direction_group"}.difference(fieldnames)
            if missing:
                raise RuntimeError(f"CSV eksik kolonlar: {', '.join(sorted(missing))}")

            for row in reader:
                parent = str(row.get("parent_segment_id", "")).strip()
                direction_group = str(row.get("direction_group", "")).strip()
                if not parent:
                    continue
                if parent.startswith(IGNORED_SEGMENT_PREFIXES):
                    continue

                key = (parent, direction_group)
                data = grouped.setdefault(
                    key,
                    {"sum_x": 0.0, "sum_y": 0.0, "count": 0},
                )
                data["sum_x"] += float(row["x"])
                data["sum_y"] += float(row["y"])
                data["count"] += 1

        segments = []
        for (parent, direction_group), data in grouped.items():
            count = data["count"]
            if count <= 0:
                continue
            segments.append(
                SegmentCandidate(
                    parent_segment_id=parent,
                    center_x=data["sum_x"] / count,
                    center_y=data["sum_y"] / count,
                    direction_group=direction_group,
                )
            )
        return segments

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg

    def on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"YOLO JSON parse edilemedi: {exc}")
            return

        detections = payload.get("detections", [])
        image_width = float(payload.get("image_width", 0.0) or 0.0)

        if not isinstance(detections, list):
            return

        for detection in detections:
            if not isinstance(detection, dict):
                continue
            self.handle_detection(detection, image_width)

    def handle_detection(self, detection: Dict[str, Any], image_width: float) -> None:
        task_id = clean_class_name(detection.get("task_id", ""))
        task_name = clean_class_name(detection.get("task_name", ""))
        class_name = clean_class_name(detection.get("class_name", ""))
        confidence = float(detection.get("confidence", 0.0) or 0.0)

        if task_id == "traffic_light" or task_name == "isik":
            if confidence < self.min_confidence:
                return
            if class_name in RED_LIGHT_CLASSES:
                self.publish_event("TRAFFIC_LIGHT_RED")
            elif class_name in GREEN_LIGHT_CLASSES:
                self.publish_event("TRAFFIC_LIGHT_GREEN")
            return

        if not (task_id == "traffic_sign" or task_name == "tabela"):
            return
        if class_name not in NO_ENTRY_CLASSES:
            return
        if confidence < self.min_confidence:
            return
        if self.pose is None:
            self.get_logger().warn("Girilmez algilandi ama /localization/pose henuz yok.")
            return

        bbox_center = detection.get("bbox_center", {})
        bbox_x = float(bbox_center.get("x", 0.0) or 0.0)
        width = image_width
        if width <= 0.0:
            width = float(detection.get("image_width", 0.0) or 0.0)
        if width <= 0.0:
            self.get_logger().warn("Girilmez algilandi ama image_width bulunamadi.")
            return

        selected = self.select_segment(bbox_x, width)
        if selected is None:
            self.get_logger().warn("Girilmez algilandi ama uygun segment bulunamadi.")
            return

        segment, score, distance, angle_error = selected
        event = f"NO_ENTRY:{segment.parent_segment_id}"
        if self.publish_event(event):
            self.get_logger().warn(
                f"NO_ENTRY localized: segment={segment.parent_segment_id} "
                f"group={segment.direction_group} score={score:.3f} "
                f"distance={distance:.3f} angle_error={math.degrees(angle_error):.1f}deg "
                f"class={class_name} conf={confidence:.2f}"
            )

    def select_segment(
        self,
        bbox_center_x: float,
        image_width: float,
    ) -> Optional[Tuple[SegmentCandidate, float, float, float]]:
        if self.pose is None:
            return None

        vehicle_x = self.pose.pose.position.x
        vehicle_y = self.pose.pose.position.y
        vehicle_yaw = yaw_from_quaternion(self.pose.pose.orientation)

        relative_x = (bbox_center_x - image_width / 2.0) / (image_width / 2.0)
        relative_x = max(-1.0, min(1.0, relative_x))
        bearing = normalize_angle(vehicle_yaw + relative_x * (self.camera_horizontal_fov / 2.0))

        best: Optional[Tuple[SegmentCandidate, float, float, float]] = None
        for segment in self.segments:
            dx = segment.center_x - vehicle_x
            dy = segment.center_y - vehicle_y
            distance = math.hypot(dx, dy)
            if distance > self.max_segment_distance:
                continue

            segment_angle = math.atan2(dy, dx)
            heading_error = abs(normalize_angle(segment_angle - vehicle_yaw))
            if heading_error > math.pi / 2.0:
                continue

            angle_error = abs(normalize_angle(segment_angle - bearing))
            score = self.angle_weight * angle_error + self.distance_weight * distance

            if best is None or score < best[1]:
                best = (segment, score, distance, angle_error)

        return best

    def publish_event(self, event: str) -> bool:
        now = time.monotonic()
        last = self.last_publish_time.get(event, 0.0)
        if now - last < self.cooldown_sec:
            return False

        self.last_publish_time[event] = now
        msg = String()
        msg.data = event
        self.event_pub.publish(msg)
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrafficSignLocalizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[TRAFFIC SIGN LOCALIZER ERROR] {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
