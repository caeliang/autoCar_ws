#!/home/ranim/autoCar_ws/.venv/bin/python3
"""
Simple Map-Based Localizer — SVD-ICP
─────────────────────────────────────
Kayıtlı PCD haritaya karşı SVD tabanlı ICP ile lokalizasyon.
Odometri ile başlangıç tahmini + sensör yükseklik ofseti desteği.

Bağımlılıklar: numpy, scipy  (open3d GEREKMEZ)
"""

import os
import sys

# Aynı dizindeki yardımcı modülü bul (install & source her ikisinde çalışır)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation

from localization_utils import (
    load_pcd, apply_transform, voxel_downsample, svd_align,
    qos_map, qos_reliable, qos_best_effort,
    make_pointcloud2, make_pose_stamped, make_transform_stamped,
)


# ══════════════════════════════════════════════════════════════════════
#  Node
# ══════════════════════════════════════════════════════════════════════

class SimpleLocalizer(Node):
    """PCD harita üzerinde SVD-ICP ile 3D lokalizasyon node'u."""

    # ──────────────────────────────────────────────────────────────────
    #  Başlatma
    # ──────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('simple_localizer')
        self._declare_parameters()
        self._load_map()
        self._init_state()
        self._create_ros_interfaces()

        self.get_logger().info('✓ Localizer ready — waiting for odometry + scan...')
        self.get_logger().info('=' * 60)

    def _declare_parameters(self):
        """ROS parametrelerini tanımla ve oku."""
        self.declare_parameter('map_pcd_path', '')
        self.declare_parameter('max_icp_dist', 5.0)
        self.declare_parameter('icp_iterations', 80)
        self.declare_parameter('icp_tolerance', 0.001)
        self.declare_parameter('sensor_z_offset', 1.0)
        self.declare_parameter('odom_topic', '/prius/odom')
        self.declare_parameter('voxel_size', 0.0)

        self.map_path     = self.get_parameter('map_pcd_path').value
        self.max_icp_dist = self.get_parameter('max_icp_dist').value
        self.icp_iters    = self.get_parameter('icp_iterations').value
        self.icp_tol      = self.get_parameter('icp_tolerance').value
        self.sensor_z     = self.get_parameter('sensor_z_offset').value
        self.odom_topic   = self.get_parameter('odom_topic').value
        self.voxel_size   = self.get_parameter('voxel_size').value

        self.get_logger().info('=' * 60)
        self.get_logger().info('  Simple Map-Based Localizer (SVD-ICP)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Map         : {self.map_path}')
        self.get_logger().info(f'Sensor Z    : {self.sensor_z}m')
        self.get_logger().info(f'Odom topic  : {self.odom_topic}')
        self.get_logger().info(f'ICP dist    : {self.max_icp_dist}m')
        self.get_logger().info(f'ICP iters   : {self.icp_iters}')

    def _load_map(self):
        """PCD haritayı yükle, istatistikleri logla, KDTree oluştur."""
        if not self.map_path or not os.path.exists(self.map_path):
            self.get_logger().error(f'Map file not found: {self.map_path}')
            raise FileNotFoundError(f'Map not found: {self.map_path}')

        self.map_points = load_pcd(self.map_path)
        self.get_logger().info(f'✓ Loaded {len(self.map_points)} map points')

        self.map_center = self.map_points.mean(axis=0)
        map_min = self.map_points.min(axis=0)
        map_max = self.map_points.max(axis=0)
        self.get_logger().info(
            f'✓ Map center: [{self.map_center[0]:.2f}, '
            f'{self.map_center[1]:.2f}, {self.map_center[2]:.2f}]')
        self.get_logger().info(
            f'✓ Map bounds: X[{map_min[0]:.1f},{map_max[0]:.1f}] '
            f'Y[{map_min[1]:.1f},{map_max[1]:.1f}] '
            f'Z[{map_min[2]:.1f},{map_max[2]:.1f}]')

        if self.voxel_size > 0:
            self.map_points = voxel_downsample(self.map_points, self.voxel_size)
            self.get_logger().info(
                f'✓ Map downsampled → {len(self.map_points)} pts '
                f'(voxel={self.voxel_size}m)')

        self.map_kdtree = KDTree(self.map_points)

    def _init_state(self):
        """Dahili durum değişkenlerini sıfırla."""
        self.last_odom      = None
        self.odom_received  = False
        self.current_T      = np.eye(4, dtype=np.float64)
        self.initialized    = False
        self.scan_count     = 0
        self.last_scan_time = None

    def _create_ros_interfaces(self):
        """Subscriber, publisher, TF broadcaster ve timer'ları oluştur."""
        # Subscribers
        self.scan_sub = self.create_subscription(
            PointCloud2, '/prius/scan',
            self.scan_callback, qos_best_effort())
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic,
            self.odom_callback, qos_best_effort())

        # Publishers
        self.map_pub      = self.create_publisher(
            PointCloud2, '/localization/map', qos_map())
        self.scan_map_pub = self.create_publisher(
            PointCloud2, '/localization/scan_in_map', qos_reliable())
        self.pose_pub     = self.create_publisher(
            PoseStamped, '/localization/pose', qos_reliable())

        # TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        self.create_timer(2.0,  self._publish_map)
        self.create_timer(0.05, self._broadcast_tf)
        self.create_timer(0.1,  self._publish_pose_timer)

    # ──────────────────────────────────────────────────────────────────
    #  Odometri
    # ──────────────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        """Odometri verisini kaydet — başlangıç tahmini ve predict step."""
        self.last_odom = msg
        if not self.odom_received:
            self.odom_received = True
            p = msg.pose.pose.position
            self.get_logger().info(
                f'✓ First odom: [{p.x:.2f}, {p.y:.2f}, {p.z:.2f}]')

    # ──────────────────────────────────────────────────────────────────
    #  Başlangıç Tahmini
    # ──────────────────────────────────────────────────────────────────

    def _initialize_pose(self, scan_arr):
        """Odometri + sensör Z ofseti ile ilk 4×4 dönüşüm tahminini oluştur."""
        T = np.eye(4, dtype=np.float64)

        if self.last_odom is not None:
            p = self.last_odom.pose.pose.position
            q = self.last_odom.pose.pose.orientation
            T[:3, 3]  = [p.x, p.y, p.z + self.sensor_z]
            T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            self.get_logger().info(
                f'✓ Initial pose from odom: '
                f'[{T[0,3]:.2f}, {T[1,3]:.2f}, {T[2,3]:.2f}]')
        else:
            T[:3, 3] = [0.0, 0.0, self.sensor_z]
            self.get_logger().warn(
                f'No odom — using origin + sensor_z: [0, 0, {self.sensor_z}]')

        _, d = self.map_kdtree.query(T[:3, 3].reshape(1, -1))
        self.get_logger().info(f'  Nearest map point: {d[0]:.2f}m away')
        if d[0] > 20.0:
            self.get_logger().warn(
                f'  ⚠ Initial guess {d[0]:.1f}m from map — ICP may struggle.')

        return T

    # ──────────────────────────────────────────────────────────────────
    #  SVD-ICP  (3 aşamalı Coarse → Fine)
    # ──────────────────────────────────────────────────────────────────

    def _run_icp(self, scan_pts):
        """3-aşamalı Coarse-to-Fine SVD-ICP çalıştır → güncel 4×4 T."""
        src = apply_transform(self.current_T, scan_pts)
        T   = self.current_T.copy()

        phases = [
            (25.0, 30),                                # çok kaba
            (10.0, self.icp_iters // 2),               # kaba
            (self.max_icp_dist, self.icp_iters),       # ince
        ]

        for idx, (max_d, max_it) in enumerate(phases):
            prev_err, converged, n_in = float('inf'), False, 0

            for _ in range(max_it):
                dists, ids = self.map_kdtree.query(src)
                mask = dists < max_d
                n_in = mask.sum()
                if n_in < 10:
                    break

                R, t = svd_align(src[mask], self.map_points[ids[mask]])

                T_inc         = np.eye(4)
                T_inc[:3, :3] = R
                T_inc[:3, 3]  = t

                src = apply_transform(T_inc, src)
                T   = T_inc @ T

                err = dists[mask].mean()
                if abs(prev_err - err) < self.icp_tol:
                    converged = True
                    break
                prev_err = err

            if self.scan_count < 30:
                self.get_logger().info(
                    f'  ICP phase {idx+1} (d<{max_d}m): '
                    f'{n_in} inliers, err={prev_err:.4f}, '
                    f'{"converged" if converged else f"max iter ({max_it})"}')

        return T

    # ──────────────────────────────────────────────────────────────────
    #  Odometri Tahmini  (Predict Step)
    # ──────────────────────────────────────────────────────────────────

    def _predict_from_odom(self):
        """Odom ile ICP başlangıç noktasını güncelle (predict step)."""
        if self.last_odom is None:
            return

        p = self.last_odom.pose.pose.position
        q = self.last_odom.pose.pose.orientation

        T_odom = np.eye(4, dtype=np.float64)
        T_odom[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T_odom[:3, 3]  = [p.x, p.y, p.z]

        T_sensor = np.eye(4, dtype=np.float64)
        T_sensor[2, 3] = self.sensor_z

        T_pred = T_odom @ T_sensor

        alpha = 0.3          # 0 → full odom,  1 → full ICP
        self.current_T[:3, 3] = (
            alpha * self.current_T[:3, 3] +
            (1 - alpha) * T_pred[:3, 3]
        )

    # ──────────────────────────────────────────────────────────────────
    #  Scan Callback  (Ana Döngü)
    # ──────────────────────────────────────────────────────────────────

    def scan_callback(self, msg):
        """Her LiDAR taramasında çağrılır — ICP + yayın."""
        self.scan_count += 1
        self.last_scan_time = msg.header.stamp

        pts = [
            [float(p[0]), float(p[1]), float(p[2])]
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                     skip_nans=True)
        ]
        if len(pts) < 10:
            return
        scan = np.array(pts, dtype=np.float64)

        # ── İlk çerçeve: başlangıç tahmini ──
        if not self.initialized:
            self.current_T   = self._initialize_pose(scan)
            self.initialized = True
            self._log_first_scan(scan)

        # ── Her 3 taramada bir ICP ──
        if self.scan_count % 3 == 0:
            if self.last_odom is not None and self.scan_count > 3:
                self._predict_from_odom()
            self.current_T = self._run_icp(scan)

            if self.scan_count % 15 == 0:
                p = self.current_T[:3, 3]
                self.get_logger().info(
                    f'ICP #{self.scan_count}: {len(pts)} pts | '
                    f'Pos: [{p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}]')

        # ── Yayınla ──
        self._publish_pose(msg.header.stamp)
        self._publish_scan_in_map(scan)

    def _log_first_scan(self, scan):
        """İlk tarama alındığında istatistikleri logla."""
        sc = scan.mean(axis=0)
        tc = apply_transform(self.current_T, scan).mean(axis=0)
        self.get_logger().info(
            f'  Scan center (sensor): '
            f'[{sc[0]:.2f}, {sc[1]:.2f}, {sc[2]:.2f}], {len(scan)} pts')
        self.get_logger().info(
            f'  Scan center (map)   : '
            f'[{tc[0]:.2f}, {tc[1]:.2f}, {tc[2]:.2f}]')
        self.get_logger().info(
            f'  Map center          : '
            f'[{self.map_center[0]:.2f}, {self.map_center[1]:.2f}, '
            f'{self.map_center[2]:.2f}]')

    # ──────────────────────────────────────────────────────────────────
    #  Yayın  (Publish)
    # ──────────────────────────────────────────────────────────────────

    def _publish_pose_timer(self):
        """Timer ile periyodik pose yayını — RViz'de ok sürekli görünsün."""
        self._publish_pose(self.get_clock().now().to_msg())

    def _publish_pose(self, stamp):
        self.pose_pub.publish(make_pose_stamped(self.current_T, 'map', stamp))

    def _publish_scan_in_map(self, scan):
        """Scan'i map çerçevesine dönüştürüp yayınla."""
        pts = apply_transform(self.current_T, scan).astype(np.float32)
        self.scan_map_pub.publish(
            make_pointcloud2(pts, 'map', self.get_clock().now().to_msg()))

    def _publish_map(self):
        """Haritayı PointCloud2 olarak yayınla."""
        if len(self.map_points) == 0:
            return
        self.map_pub.publish(
            make_pointcloud2(
                self.map_points.astype(np.float32),
                'map', self.get_clock().now().to_msg()))

    def _broadcast_tf(self):
        """map → sensor_link TF'ini yayınla."""
        if self.last_scan_time is None:
            return
        self.tf_broadcaster.sendTransform(
            make_transform_stamped(
                self.current_T, 'map', 'sensor_link',
                self.get_clock().now().to_msg()))


# ══════════════════════════════════════════════════════════════════════
#  main
# ══════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SimpleLocalizer()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()