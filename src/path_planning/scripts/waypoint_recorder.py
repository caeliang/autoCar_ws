#!/usr/bin/env python3
"""
Waypoint Recorder — otomatik waypoint kayıt aracı.

Localization çalışırken bu scripti başlat:
  Araç min_distance (varsayılan 0.5 m) hareket ettiğinde otomatik kayıt yapar.

  Tuşlar:
  - d      → son waypoint'i sil
  - l      → kayıtlı waypoint'leri listele
  - s      → dosyaya kaydet
  - q      → kaydet ve çık
  - p      → otomatik kaydı duraklat / devam ettir
  - ENTER  → manuel waypoint ekle (duraklatılmışsa da çalışır)

/localization/pose topic'inden konum alır (map frame).
"""

import sys
import os
import math
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        # Parameters
        self.declare_parameter('output_file', '')
        self.declare_parameter('min_distance', 0.5)

        self.output_file = self.get_parameter('output_file').value
        self.min_dist = self.get_parameter('min_distance').value

        if not self.output_file:
            wp_dir = os.path.expanduser('~/autoCar_ws/waypoints')
            os.makedirs(wp_dir, exist_ok=True)
            from datetime import datetime
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.output_file = os.path.join(wp_dir, f'route_{ts}.yaml')

        self.waypoints = []
        self.current_pose = None
        self.pose_count = 0
        self.auto_record = True   # otomatik kayıt varsayılan açık

        # Subscriber — Best Effort (Gazebo + localizer uyumlu)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/localization/pose',
            self.pose_cb, qos)

        # Marker publisher — kayıt sırasında RViz'de noktaları göster
        self.marker_pub = self.create_publisher(
            MarkerArray, '/waypoint_recorder/markers', 10)
        self.create_timer(1.0, self.publish_markers)

        self.get_logger().info('━' * 50)
        self.get_logger().info('  WAYPOINT RECORDER — OTOMATİK MOD')
        self.get_logger().info('━' * 50)
        self.get_logger().info(f'  Çıktı: {self.output_file}')
        self.get_logger().info(f'  Min mesafe: {self.min_dist} m')
        self.get_logger().info(f'  Otomatik kayıt: AÇIK')
        self.get_logger().info('━' * 50)
        self.get_logger().info('  Araç hareket ettikçe otomatik kaydeder.')
        self.get_logger().info('  ENTER → manuel ekle  |  d → sil')
        self.get_logger().info('  l → listele  |  s → kaydet')
        self.get_logger().info('  p → duraklat/devam  |  q → kaydet+çık')
        self.get_logger().info('━' * 50)
        print()

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg
        self.pose_count += 1

        # Otomatik kayıt — her pose geldiğinde mesafe kontrol et
        if self.auto_record:
            self._try_auto_record()

    def _distance_to_last(self, p):
        """Mevcut konum ile son waypoint arasındaki mesafe."""
        if not self.waypoints:
            return float('inf')
        last = self.waypoints[-1]
        dx = p.x - last['x']
        dy = p.y - last['y']
        return math.sqrt(dx * dx + dy * dy)

    def _try_auto_record(self):
        """Otomatik: son waypoint'ten min_dist kadar uzaklaştıysa kaydet."""
        if self.current_pose is None:
            return
        p = self.current_pose.pose.position
        dist = self._distance_to_last(p)
        if dist >= self.min_dist:
            self._record_current('AUTO')

    def _record_current(self, tag=''):
        """Mevcut konumu waypoint olarak ekle."""
        if self.current_pose is None:
            print('  ✗ Henüz pose alınmadı — localization çalışıyor mu?')
            return
        p = self.current_pose.pose.position
        q = self.current_pose.pose.orientation

        # Quaternion → yaw
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        wp = {
            'x': round(p.x, 4),
            'y': round(p.y, 4),
            'yaw': round(yaw, 6),
        }
        self.waypoints.append(wp)
        idx = len(self.waypoints) - 1
        label = f'[{tag}] ' if tag else ''
        print(f'  ✓ {label}Waypoint #{idx}: ({wp["x"]:.2f}, {wp["y"]:.2f})  '
              f'| toplam {len(self.waypoints)}')

    def add_waypoint(self):
        """Manuel ENTER ile ekleme."""
        if self.current_pose is None:
            print('  ✗ Henüz pose alınmadı — localization çalışıyor mu?')
            return
        p = self.current_pose.pose.position
        dist = self._distance_to_last(p)
        if dist < 0.1:
            print(f'  ✗ Son waypoint\'e çok yakın ({dist:.2f} m) — biraz hareket et')
            return
        self._record_current('MANUEL')

    def delete_last(self):
        if not self.waypoints:
            print('  ✗ Silinecek waypoint yok')
            return
        removed = self.waypoints.pop()
        print(f'  ✗ Waypoint #{len(self.waypoints)} silindi: '
              f'({removed["x"]:.2f}, {removed["y"]:.2f})')

    def list_waypoints(self):
        if not self.waypoints:
            print('  (boş)')
            return
        print(f'\n  ── {len(self.waypoints)} Waypoint ──')
        for i, wp in enumerate(self.waypoints):
            print(f'  [{i:3d}]  x={wp["x"]:8.3f}  y={wp["y"]:8.3f}  '
                  f'yaw={wp["yaw"]:6.3f}')
        print()

    def save_to_file(self):
        if not self.waypoints:
            print('  ✗ Kaydedilecek waypoint yok')
            return False

        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)

        with open(self.output_file, 'w') as f:
            f.write('# Waypoint file — generated by waypoint_recorder\n')
            f.write(f'# Frame: map\n')
            f.write(f'# Total: {len(self.waypoints)} waypoints\n')
            f.write(f'# Format: x, y, yaw (radyan)\n\n')
            f.write('waypoints:\n')
            for wp in self.waypoints:
                f.write(f'  - x: {wp["x"]}\n')
                f.write(f'    y: {wp["y"]}\n')
                f.write(f'    yaw: {wp["yaw"]}\n\n')

        print(f'  💾 {len(self.waypoints)} waypoint kaydedildi → {self.output_file}')
        return True

    def toggle_auto(self):
        """Otomatik kaydı aç/kapa."""
        self.auto_record = not self.auto_record
        state = 'AÇIK ▶' if self.auto_record else 'DURAKLATILDI ⏸'
        print(f'  ⚡ Otomatik kayıt: {state}')

    def publish_markers(self):
        """RViz'de kayıtlı waypoint'leri küre olarak göster."""
        if not self.waypoints:
            return

        ma = MarkerArray()

        # Silme marker'ı
        delete = Marker()
        delete.header.frame_id = 'map'
        delete.header.stamp = self.get_clock().now().to_msg()
        delete.ns = 'rec_wp'
        delete.action = Marker.DELETEALL
        ma.markers.append(delete)

        for i, wp in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'rec_wp'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp['x']
            m.pose.position.y = wp['y']
            m.pose.position.z = wp['z'] + 0.3
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 0.4
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.3
            m.color.a = 0.9
            ma.markers.append(m)

            # Numara yazısı
            txt = Marker()
            txt.header = m.header
            txt.ns = 'rec_wp_txt'
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = wp['x']
            txt.pose.position.y = wp['y']
            txt.pose.position.z = wp['z'] + 0.8
            txt.scale.z = 0.35
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 1.0
            txt.color.a = 1.0
            txt.text = str(i)
            ma.markers.append(txt)

        # Bağlantı çizgisi
        if len(self.waypoints) > 1:
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = self.get_clock().now().to_msg()
            line.ns = 'rec_wp_line'
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.color.r = 0.2
            line.color.g = 0.8
            line.color.b = 1.0
            line.color.a = 0.7
            from geometry_msgs.msg import Point
            for wp in self.waypoints:
                pt = Point()
                pt.x = wp['x']
                pt.y = wp['y']
                pt.z = wp['z'] + 0.1
                line.points.append(pt)
            ma.markers.append(line)

        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = WaypointRecorder()

    # Terminal ayarları (non-blocking keyboard input)
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while rclpy.ok():
            # ROS spin (non-blocking)
            rclpy.spin_once(node, timeout_sec=0.1)

            # Keyboard check (non-blocking)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                ch = sys.stdin.read(1)

                if ch == '\n' or ch == '\r':
                    node.add_waypoint()
                elif ch == 'd' or ch == 'D':
                    node.delete_last()
                elif ch == 'l' or ch == 'L':
                    node.list_waypoints()
                elif ch == 's' or ch == 'S':
                    node.save_to_file()
                elif ch == 'p' or ch == 'P':
                    node.toggle_auto()
                elif ch == 'q' or ch == 'Q':
                    node.save_to_file()
                    print('\n  Çıkılıyor...')
                    break

            # Pose durumu göster (her 100 mesajda bir)
            if node.pose_count > 0 and node.pose_count % 100 == 0:
                if node.current_pose:
                    p = node.current_pose.pose.position
                    auto_str = '▶ OTO' if node.auto_record else '⏸ DURDURULDU'
                    print(f'  📍 ({p.x:.2f}, {p.y:.2f})  '
                          f'| {len(node.waypoints)} wp  '
                          f'| {auto_str}          ', end='\r')

    except KeyboardInterrupt:
        print('\n  Ctrl+C — kaydediliyor...')
        node.save_to_file()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
