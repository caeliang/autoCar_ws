#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import csv
import sys
import os
import math

class WaypointVisualizer(Node):
    def __init__(self, csv_path):
        super().__init__('waypoint_visualizer')
        self.publisher = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        
        if not os.path.exists(csv_path):
            self.get_logger().error(f"CSV dosyası bulunamadı: {csv_path}")
            return

        self.waypoints = self.load_waypoints(csv_path)
        self.get_logger().info(f"{len(self.waypoints)} waypoint yüklendi. Görselleştiriliyor...")
        
        # Timer ile sürekli yayınla (RViz'de kaybolmaması için)
        self.timer = self.create_timer(1.0, self.publish_markers)

    def load_waypoints(self, path):
        waypoints = []
        try:
            with open(path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    waypoints.append({
                        'x': float(row['x']),
                        'y': float(row['y']),
                        'z': float(row.get('z', 0.0)),
                        'yaw': float(row.get('yaw', 0.0))
                    })
        except Exception as e:
            self.get_logger().error(f"CSV okuma hatası: {e}")
        return waypoints

    def publish_markers(self):
        marker_array = MarkerArray()
        
        # 1. Noktalar için Sphere List (Küre Listesi)
        sphere_marker = Marker()
        sphere_marker.header.frame_id = "map"
        sphere_marker.header.stamp = self.get_clock().now().to_msg()
        sphere_marker.ns = "waypoints"
        sphere_marker.id = 0
        sphere_marker.type = Marker.SPHERE_LIST
        sphere_marker.action = Marker.ADD
        sphere_marker.pose.orientation.w = 1.0
        sphere_marker.scale.x = 0.3
        sphere_marker.scale.y = 0.3
        sphere_marker.scale.z = 0.3
        sphere_marker.color.a = 1.0
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 1.0
        sphere_marker.color.b = 0.0

        # 2. Yönler için Arrow (Ok) Listesi (Seyrek olması için her 5 noktada bir)
        for i, wp in enumerate(self.waypoints):
            p = Point()
            p.x = wp['x']
            p.y = wp['y']
            p.z = wp['z']
            sphere_marker.points.append(p)

            if i % 5 == 0:
                arrow_marker = Marker()
                arrow_marker.header.frame_id = "map"
                arrow_marker.header.stamp = sphere_marker.header.stamp
                arrow_marker.ns = "orientations"
                arrow_marker.id = i + 1
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                
                arrow_marker.pose.position.x = wp['x']
                arrow_marker.pose.position.y = wp['y']
                arrow_marker.pose.position.z = wp['z'] + 0.1
                
                # Yaw to Quaternion
                arrow_marker.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
                arrow_marker.pose.orientation.w = math.cos(wp['yaw'] / 2.0)
                
                arrow_marker.scale.x = 0.5 # Uzunluk
                arrow_marker.scale.y = 0.1 # Genişlik
                arrow_marker.scale.z = 0.1 # Yükseklik
                
                arrow_marker.color.a = 1.0
                arrow_marker.color.r = 1.0
                arrow_marker.color.g = 1.0
                arrow_marker.color.b = 0.0
                marker_array.markers.append(arrow_marker)

        marker_array.markers.append(sphere_marker)
        self.publisher.publish(marker_array)

def main():
    if len(sys.argv) < 2:
        print("Kullanım: python3 waypoint_visualizer.py <csv_dosya_yolu>")
        return

    rclpy.init()
    node = WaypointVisualizer(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
