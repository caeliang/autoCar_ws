#!/usr/bin/env python3
"""
Open3D Map-Based Localizer — Point-to-Plane ICP
────────────────────────────────────────────────
Kayıtlı PCD haritaya karşı Open3D ICP ile lokalizasyon.
Modüler backend sistemi: ICP, NDT, GICP eklenebilir.

Bağımlılıklar: open3d, scipy
"""

import sys
import os

# venv + yardımcı modül yolu
venv_path = os.path.expanduser('~/autoCar_ws/.venv/lib/python3.10/site-packages')
if os.path.exists(venv_path):
    sys.path.insert(0, venv_path)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation, Slerp

from localization_utils import (
    make_pointcloud2, make_pose_with_covariance, make_transform_stamped,
)


# ══════════════════════════════════════════════════════════════════════
#  ICP Backend'leri  (modüler — yeni algoritma eklemek kolay)
# ══════════════════════════════════════════════════════════════════════

class LocalizationBackend:
    """Lokalizasyon algoritması temel sınıfı."""

    def __init__(self, map_cloud):
        self.map_cloud = map_cloud

    def localize(self, scan_cloud, initial_pose):
        """Dönüş: (4×4 transformation, fitness skoru)"""
        raise NotImplementedError


class ICPBackend(LocalizationBackend):
    """Open3D Point-to-Plane ICP."""

    def __init__(self, map_cloud, max_correspondence_distance=0.5):
        super().__init__(map_cloud)
        self.max_dist = max_correspondence_distance

    def localize(self, scan_cloud, initial_pose):
        reg = o3d.pipelines.registration.registration_icp(
            scan_cloud, self.map_cloud,
            self.max_dist, initial_pose,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=50,
            ),
        )
        return reg.transformation, reg.fitness


# ══════════════════════════════════════════════════════════════════════
#  Node
# ══════════════════════════════════════════════════════════════════════

class MapBasedLocalizer(Node):
    """Open3D ICP ile harita tabanlı 3D lokalizasyon node'u."""

    # ──────────────────────────────────────────────────────────────────
    #  Başlatma
    # ──────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('map_based_localizer')
        self._declare_parameters()
        self._load_map()
        self._init_backend()
        self._init_state()
        self._create_ros_interfaces()

        self.get_logger().info('✓ Localization node ready!')
        self.get_logger().info('=' * 60)

    def _declare_parameters(self):
        """ROS parametrelerini tanımla ve oku."""
        self.declare_parameter('map_pcd_path', '')
        self.declare_parameter('localization_method', 'icp')
        self.declare_parameter('scan_topic', '/prius/scan')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('max_correspondence_distance', 5.0)
        self.declare_parameter('voxel_size', 0.3)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('fitness_threshold', 0.05)
        self.declare_parameter('smoothing_factor', 0.3)

        self.map_path       = self.get_parameter('map_pcd_path').value
        self.method         = self.get_parameter('localization_method').value
        self.scan_topic     = self.get_parameter('scan_topic').value
        self.odom_topic     = self.get_parameter('odom_topic').value
        self.max_dist       = self.get_parameter('max_correspondence_distance').value
        self.voxel_size     = self.get_parameter('voxel_size').value
        self.fitness_thresh = self.get_parameter('fitness_threshold').value
        self.smooth         = self.get_parameter('smoothing_factor').value

        self.get_logger().info('=' * 60)
        self.get_logger().info('  Map-Based 3D Localization Node')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Map         : {self.map_path}')
        self.get_logger().info(f'Method      : {self.method.upper()}')
        self.get_logger().info(f'Scan topic  : {self.scan_topic}')
        self.get_logger().info(f'Voxel size  : {self.voxel_size}m')
        self.get_logger().info(f'Fitness thr : {self.fitness_thresh}')
        self.get_logger().info(f'Smoothing   : {self.smooth}')

    def _load_map(self):
        """PCD haritayı Open3D ile yükle, normal hesapla."""
        if not self.map_path or not os.path.exists(self.map_path):
            self.get_logger().error(f'Map not found: {self.map_path}')
            raise FileNotFoundError(f'Map PCD not found: {self.map_path}')

        self.get_logger().info('Loading map...')
        self.map_cloud = o3d.io.read_point_cloud(self.map_path)
        self.map_cloud = self.map_cloud.voxel_down_sample(self.voxel_size)

        self.get_logger().info('  Computing normals...')
        self.map_cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=1.0, max_nn=30))

        pts = np.asarray(self.map_cloud.points)
        c = pts.mean(axis=0)
        self.get_logger().info(f'  ✓ {len(pts)} points')
        self.get_logger().info(
            f'  Bounds: X[{pts[:,0].min():.2f},{pts[:,0].max():.2f}] '
            f'Y[{pts[:,1].min():.2f},{pts[:,1].max():.2f}] '
            f'Z[{pts[:,2].min():.2f},{pts[:,2].max():.2f}]')
        self.get_logger().info(
            f'  Center: [{c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f}]')

    def _init_backend(self):
        """Seçilen lokalizasyon algoritmasını oluştur."""
        if self.method == 'icp':
            self.backend = ICPBackend(self.map_cloud, self.max_dist)
        else:
            raise ValueError(f'Unknown localization method: {self.method}')

    def _init_state(self):
        """Dahili durum değişkenlerini sıfırla."""
        self.current_pose       = np.eye(4)
        self.last_odom          = None
        self.localization_count = 0
        self.last_fitness       = 0.0

    def _create_ros_interfaces(self):
        """Subscriber, publisher, TF broadcaster ve timer oluştur."""
        self.scan_sub = self.create_subscription(
            PointCloud2, self.scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)

        self.pose_pub      = self.create_publisher(
            PoseWithCovarianceStamped, '/localization/pose', 10)
        self.map_cloud_pub = self.create_publisher(
            PointCloud2, '/localization/map_cloud', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(5.0, self._publish_map_timer)

    # ──────────────────────────────────────────────────────────────────
    #  Callbacks
    # ──────────────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        """Son odometriyi kaydet — initial guess için."""
        self.last_odom = msg

    def scan_callback(self, msg):
        """Ana lokalizasyon döngüsü — her scan geldiğinde çağrılır."""
        try:
            scan_cloud = self._msg_to_open3d(msg)
            if scan_cloud is None:
                return

            self._log_scan_debug(scan_cloud)
            self._set_initial_guess(scan_cloud)

            # ── ICP çalıştır ──
            transformation, fitness = self.backend.localize(
                scan_cloud, self.current_pose.copy())
            transformation = transformation.copy()

            self.get_logger().info(
                f'ICP #{self.localization_count}: fitness={fitness:.3f}, '
                f'pos=[{transformation[0,3]:.2f}, {transformation[1,3]:.2f}, '
                f'{transformation[2,3]:.2f}]',
                throttle_duration_sec=1.0)

            # ── Fitness kontrolü ──
            if fitness < self.fitness_thresh:
                self.localization_count += 1
                self.get_logger().warn(
                    f'Low fitness {fitness:.3f} — skipping',
                    throttle_duration_sec=2.0)
                if self.localization_count <= 5:
                    self.get_logger().error(
                        'ICP not converging! Check coordinate frames / overlap.')
                return

            # ── Smoothing ──
            if self.localization_count > 0:
                transformation = self._smooth(transformation)

            # ── Güncelle ve yayınla ──
            self.current_pose = transformation
            self.last_fitness = fitness
            self.localization_count += 1

            self.pose_pub.publish(
                make_pose_with_covariance(
                    transformation, 'map', msg.header.stamp, fitness))
            self.tf_broadcaster.sendTransform(
                make_transform_stamped(
                    transformation, 'map', 'odom', msg.header.stamp))

            self.get_logger().info(
                f'Localized | fitness={fitness:.3f} | '
                f'#{self.localization_count}',
                throttle_duration_sec=1.0)

        except Exception as e:
            self.get_logger().error(f'Localization failed: {e}')

    # ──────────────────────────────────────────────────────────────────
    #  Yardımcı Metodlar
    # ──────────────────────────────────────────────────────────────────

    def _msg_to_open3d(self, msg):
        """PointCloud2 → Open3D PointCloud  (None if too few points)."""
        points = [
            [float(p[0]), float(p[1]), float(p[2])]
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                     skip_nans=True)
        ]
        if len(points) < 20:
            self.get_logger().warn(
                f'Too few points: {len(points)}',
                throttle_duration_sec=5.0)
            return None

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(
            np.array(points, dtype=np.float64))

        if len(cloud.points) < 50:
            self.get_logger().warn(
                f'Sparse scan: {len(cloud.points)} pts',
                throttle_duration_sec=2.0)
        return cloud

    def _log_scan_debug(self, cloud):
        """İlk birkaç taramada istatistikleri logla."""
        if self.localization_count >= 3:
            return
        pts = np.asarray(cloud.points)
        mn, mx, c = pts.min(0), pts.max(0), pts.mean(0)
        self.get_logger().info(
            f'Scan #{self.localization_count}: {len(pts)} points | '
            f'Center: [{c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f}] | '
            f'Bounds: X[{mn[0]:.2f},{mx[0]:.2f}] '
            f'Y[{mn[1]:.2f},{mx[1]:.2f}] '
            f'Z[{mn[2]:.2f},{mx[2]:.2f}]')

    def _set_initial_guess(self, cloud):
        """İlk iterasyonda harita merkezini başlangıç tahmini yap."""
        if self.localization_count == 0:
            center = np.asarray(
                self.map_cloud.get_center(), dtype=np.float64).copy()
            self.current_pose[:3, 3] = center
            sc = np.asarray(cloud.points).mean(0)
            self.get_logger().info(
                f'First iteration: pose=Map center '
                f'[{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}], '
                f'Scan center=[{sc[0]:.2f}, {sc[1]:.2f}, {sc[2]:.2f}]')

    def _smooth(self, T):
        """Exponential smoothing — pozisyon + SLERP rotasyon."""
        # Pozisyon
        T[:3, 3] = (
            self.smooth * self.current_pose[:3, 3] +
            (1 - self.smooth) * T[:3, 3])

        # Rotasyon (SLERP)
        r_old = Rotation.from_matrix(self.current_pose[:3, :3])
        r_new = Rotation.from_matrix(T[:3, :3])
        slerp = Slerp([0, 1], Rotation.concatenate([r_old, r_new]))
        T[:3, :3] = slerp(1 - self.smooth).as_matrix()

        return T

    # ──────────────────────────────────────────────────────────────────
    #  Harita Yayını
    # ──────────────────────────────────────────────────────────────────

    def _publish_map_timer(self):
        """Haritayı periyodik olarak yayınla (RViz görselleştirme)."""
        pts = np.asarray(self.map_cloud.points)
        self.map_cloud_pub.publish(
            make_pointcloud2(
                pts.astype(np.float32), 'sensor_link',
                self.get_clock().now().to_msg()))


# ══════════════════════════════════════════════════════════════════════
#  main
# ══════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(MapBasedLocalizer())
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()