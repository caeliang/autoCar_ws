#!/usr/bin/env python3
import json
import os
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloGazeboDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_gazebo_detector")

        self.declare_parameter("traffic_light_model_path", "/home/ranim/autoCar_ws/isik_modeli.pt")
        self.declare_parameter("traffic_sign_model_path", "/home/ranim/autoCar_ws/tabela_modeli (1).pt")
        self.declare_parameter("direction_model_path", "/home/ranim/autoCar_ws/solsag_modeli (1).pt")
        self.declare_parameter("image_topic", "/prius/front_camera_sensor/image_raw")
        self.declare_parameter("annotated_topic", "/perception/yolo/annotated_image")
        self.declare_parameter("detections_topic", "/perception/yolo/detections")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("device", "")
        self.declare_parameter("max_detections", 100)
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("process_every_n_frames", 1)

        self.traffic_light_model_path = self.get_parameter(
            "traffic_light_model_path"
        ).get_parameter_value().string_value
        self.traffic_sign_model_path = self.get_parameter(
            "traffic_sign_model_path"
        ).get_parameter_value().string_value
        self.direction_model_path = self.get_parameter(
            "direction_model_path"
        ).get_parameter_value().string_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.annotated_topic = self.get_parameter("annotated_topic").get_parameter_value().string_value
        self.detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value

        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.max_detections = int(self.get_parameter("max_detections").value)
        self.publish_annotated_image = bool(self.get_parameter("publish_annotated_image").value)
        self.process_every_n_frames = max(
            1, int(self.get_parameter("process_every_n_frames").value)
        )

        self.bridge = CvBridge()
        self.frame_count = 0

        self.task_model_configs = [
            {
                "task_id": "traffic_light",
                "task_name": "isik",
                "model_path": self.traffic_light_model_path,
                "color": (0, 0, 255),
            },
            {
                "task_id": "traffic_sign",
                "task_name": "tabela",
                "model_path": self.traffic_sign_model_path,
                "color": (255, 140, 0),
            },
            {
                "task_id": "direction",
                "task_name": "sol_sag",
                "model_path": self.direction_model_path,
                "color": (0, 220, 0),
            },
        ]
        self.task_colors = {
            str(config["task_id"]): config["color"] for config in self.task_model_configs
        }
        self.detectors = self.load_detectors()

        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)

        self.annotated_pub: Optional[Any] = None
        if self.publish_annotated_image:
            self.annotated_pub = self.create_publisher(Image, self.annotated_topic, 10)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"Subscribing image topic: {self.image_topic}")
        self.get_logger().info(f"Publishing detections: {self.detections_topic}")
        self.get_logger().info(
            f"Active specialized models: {', '.join(detector['task_name'] for detector in self.detectors)}"
        )

    def load_detectors(self) -> List[Dict[str, Any]]:
        detectors: List[Dict[str, Any]] = []

        for config in self.task_model_configs:
            model_path = os.path.expanduser(str(config["model_path"]))

            if not model_path:
                self.get_logger().warning(
                    f"Task {config['task_name']} skipped because model path is empty."
                )
                continue

            if not os.path.isfile(model_path):
                self.get_logger().warning(
                    f"Task {config['task_name']} skipped. Model file not found: {model_path}"
                )
                continue

            model = self.load_model(model_path)
            detector = {
                "task_id": config["task_id"],
                "task_name": config["task_name"],
                "model_path": model_path,
                "model": model,
                "names": getattr(model, "names", {}),
                "color": config["color"],
            }
            detectors.append(detector)
            self.get_logger().info(
                f"YOLO model loaded for task {config['task_name']}: {model_path}"
            )

        if not detectors:
            raise RuntimeError("No valid YOLO model could be loaded for any task.")

        return detectors

    def load_model(self, model_path: str) -> Any:
        if not model_path:
            raise RuntimeError(
                "Model path is empty. Please provide a valid model path."
            )

        model_path = os.path.expanduser(model_path)

        if not os.path.isfile(model_path):
            raise RuntimeError(f"Model dosyası bulunamadı: {model_path}")

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics kurulu değil. Kurulum: python3 -m pip install ultralytics"
            ) from exc

        return YOLO(model_path)

    def image_callback(self, msg: Image) -> None:
        self.frame_count += 1

        if self.frame_count % self.process_every_n_frames != 0:
            return

        started = time.perf_counter()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Görüntü OpenCV formatına çevrilemedi: {exc}")
            return

        detections: List[Dict[str, Any]] = []
        inference_per_task_ms: Dict[str, float] = {}

        for detector in self.detectors:
            task_started = time.perf_counter()

            try:
                results = detector["model"].predict(
                    source=frame,
                    conf=self.confidence_threshold,
                    iou=self.iou_threshold,
                    imgsz=self.image_size,
                    device=self.device if self.device else None,
                    max_det=self.max_detections,
                    verbose=False,
                )
            except Exception as exc:
                self.get_logger().error(
                    f"YOLO inference hatasi ({detector['task_name']}): {exc}"
                )
                continue

            inference_per_task_ms[detector["task_id"]] = round(
                (time.perf_counter() - task_started) * 1000.0,
                2,
            )

            if not results:
                continue

            detections.extend(self.parse_detections(results[0], detector))

        inference_ms = (time.perf_counter() - started) * 1000.0

        self.publish_detections(msg, detections, inference_ms, inference_per_task_ms)

        if self.annotated_pub is not None:
            annotated = self.draw_detections(frame, detections)
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)

    def parse_detections(self, result: Any, detector: Dict[str, Any]) -> List[Dict[str, Any]]:
        detections: List[Dict[str, Any]] = []

        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return detections

        xyxy = boxes.xyxy.cpu().numpy()
        confidences = boxes.conf.cpu().numpy()
        class_ids = boxes.cls.cpu().numpy().astype(int)

        for bbox, confidence, class_id in zip(xyxy, confidences, class_ids):
            x1, y1, x2, y2 = [float(v) for v in bbox]

            detections.append(
                {
                    "task_id": detector["task_id"],
                    "task_name": detector["task_name"],
                    "model_file": os.path.basename(str(detector["model_path"])),
                    "class_id": int(class_id),
                    "class_name": self.class_name(int(class_id), detector["names"]),
                    "confidence": round(float(confidence), 4),
                    "bbox_xyxy": {
                        "x1": round(x1, 2),
                        "y1": round(y1, 2),
                        "x2": round(x2, 2),
                        "y2": round(y2, 2),
                    },
                    "bbox_center": {
                        "x": round((x1 + x2) / 2.0, 2),
                        "y": round((y1 + y2) / 2.0, 2),
                        "width": round(x2 - x1, 2),
                        "height": round(y2 - y1, 2),
                    },
                }
            )

        return detections

    def class_name(self, class_id: int, names: Any) -> str:
        if isinstance(names, dict):
            return str(names.get(class_id, class_id))

        if isinstance(names, list) and 0 <= class_id < len(names):
            return str(names[class_id])

        return str(class_id)

    def publish_detections(
        self,
        image_msg: Image,
        detections: List[Dict[str, Any]],
        inference_ms: float,
        inference_per_task_ms: Dict[str, float],
    ) -> None:
        per_task_count: Dict[str, int] = {}
        for detection in detections:
            task_name = str(detection.get("task_name", "unknown"))
            per_task_count[task_name] = per_task_count.get(task_name, 0) + 1

        payload = {
            "stamp": {
                "sec": int(image_msg.header.stamp.sec),
                "nanosec": int(image_msg.header.stamp.nanosec),
            },
            "frame_id": image_msg.header.frame_id,
            "image_width": int(image_msg.width),
            "image_height": int(image_msg.height),
            "inference_ms": round(inference_ms, 2),
            "inference_per_task_ms": inference_per_task_ms,
            "count": len(detections),
            "count_per_task": per_task_count,
            "detections": detections,
        }

        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.detections_pub.publish(out)

        if detections:
            detected_objects = ", ".join(
                f"{detection['task_name']}/{detection['class_name']} ({detection['confidence']:.2f})"
                for detection in detections
            )
            self.get_logger().info(f"Algilanan nesneler: {detected_objects}")

    def draw_detections(self, frame: Any, detections: List[Dict[str, Any]]) -> Any:
        annotated = frame.copy()

        for detection in detections:
            bbox = detection["bbox_xyxy"]

            x1 = int(round(bbox["x1"]))
            y1 = int(round(bbox["y1"]))
            x2 = int(round(bbox["x2"]))
            y2 = int(round(bbox["y2"]))

            task_id = str(detection.get("task_id", ""))
            color = self.task_colors.get(task_id, (0, 220, 0))
            label = (
                f"{detection['task_name']}:{detection['class_name']} "
                f"{detection['confidence']:.2f}"
            )

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            text_size, baseline = cv2.getTextSize(
                label,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                2,
            )

            label_y1 = max(0, y1 - text_size[1] - baseline - 6)
            label_y2 = max(text_size[1] + baseline + 6, y1)

            cv2.rectangle(
                annotated,
                (x1, label_y1),
                (x1 + text_size[0] + 8, label_y2),
                color,
                -1,
            )

            cv2.putText(
                annotated,
                label,
                (x1 + 4, label_y2 - baseline - 3),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                2,
                cv2.LINE_AA,
            )

        return annotated


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = YoloGazeboDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[YOLO NODE ERROR] {exc}")
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()