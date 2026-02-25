#!/usr/bin/env python3
"""
Lokalizasyon Ortak Yardımcı Modülü
───────────────────────────────────
PCD dosya işlemleri, nokta bulutu dönüşümleri,
QoS profilleri ve ROS 2 mesaj fabrikaları.

Bu modül hem simple_localizer hem map_based_localizer
tarafından ortaklaşa kullanılır.
"""

import numpy as np
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation


# ══════════════════════════════════════════════════════════════════════
#  PCD Dosya İşlemleri
# ══════════════════════════════════════════════════════════════════════

def load_pcd(file_path: str) -> np.ndarray:
    """ASCII PCD dosyasını oku → Nx3 float64 numpy dizisi."""
    points = []
    with open(file_path, 'r') as f:
        data_started = False
        for line in f:
            if line.startswith('DATA'):
                data_started = True
                continue
            if data_started:
                try:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        points.append([
                            float(parts[0]),
                            float(parts[1]),
                            float(parts[2]),
                        ])
                except Exception:
                    pass
    return np.array(points, dtype=np.float64)


# ══════════════════════════════════════════════════════════════════════
#  Nokta Bulutu Dönüşümleri
# ══════════════════════════════════════════════════════════════════════

def apply_transform(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """4×4 homojen dönüşüm matrisini Nx3 noktalara uygula → Nx3."""
    h = np.hstack([pts, np.ones((len(pts), 1))])
    return (T @ h.T).T[:, :3]


def voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    """Basit voxel-grid downsampling."""
    quantized = np.round(pts / voxel_size).astype(np.int32)
    _, idx = np.unique(quantized, axis=0, return_index=True)
    return pts[idx]


def svd_align(src: np.ndarray, tgt: np.ndarray):
    """
    SVD ile en-küçük-kareler rigid hizalama.

    Dönüş
    ------
    R : 3×3 rotasyon matrisi
    t : 3   öteleme vektörü
        tgt ≈ R @ src + t
    """
    sc = src.mean(axis=0)
    tc = tgt.mean(axis=0)
    H  = (src - sc).T @ (tgt - tc)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:          # yansıma düzeltmesi
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = tc - R @ sc
    return R, t


# ══════════════════════════════════════════════════════════════════════
#  QoS Profil Fabrikaları
# ══════════════════════════════════════════════════════════════════════

def qos_map(depth: int = 1) -> QoSProfile:
    """Harita yayını — Transient Local (geç bağlanan RViz bile alır)."""
    return QoSProfile(
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def qos_reliable(depth: int = 5) -> QoSProfile:
    """Scan / pose yayını — Volatile + Reliable (RViz uyumlu)."""
    return QoSProfile(
        durability=DurabilityPolicy.VOLATILE,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def qos_best_effort(depth: int = 5) -> QoSProfile:
    """Gazebo sensör girişleri — Volatile + Best Effort."""
    return QoSProfile(
        durability=DurabilityPolicy.VOLATILE,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ══════════════════════════════════════════════════════════════════════
#  ROS 2 Mesaj Fabrikaları
# ══════════════════════════════════════════════════════════════════════

def make_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """
    Nx3 float32 diziden PointCloud2 mesajı oluştur.
    *points* otomatik olarak float32'ye dönüştürülür.
    """
    pts = np.ascontiguousarray(points, dtype=np.float32)
    msg = PointCloud2()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    msg.height          = 1
    msg.width           = len(pts)
    msg.fields = [
        pc2.PointField(name='x', offset=0,  datatype=pc2.PointField.FLOAT32, count=1),
        pc2.PointField(name='y', offset=4,  datatype=pc2.PointField.FLOAT32, count=1),
        pc2.PointField(name='z', offset=8,  datatype=pc2.PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step   = 12
    msg.row_step     = 12 * msg.width
    msg.is_dense     = True
    msg.data         = pts.tobytes()
    return msg


def make_pose_stamped(T: np.ndarray, frame_id: str, stamp) -> PoseStamped:
    """4×4 dönüşüm matrisinden PoseStamped oluştur."""
    pos = T[:3, 3]
    q   = Rotation.from_matrix(T[:3, :3]).as_quat()   # [x, y, z, w]

    msg = PoseStamped()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x    = float(pos[0])
    msg.pose.position.y    = float(pos[1])
    msg.pose.position.z    = float(pos[2])
    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])
    return msg


def make_transform_stamped(T: np.ndarray, frame_id: str,
                           child_frame_id: str, stamp) -> TransformStamped:
    """4×4 dönüşüm matrisinden TransformStamped oluştur."""
    pos = T[:3, 3]
    q   = Rotation.from_matrix(T[:3, :3]).as_quat()

    t = TransformStamped()
    t.header.stamp    = stamp
    t.header.frame_id = frame_id
    t.child_frame_id  = child_frame_id
    t.transform.translation.x = float(pos[0])
    t.transform.translation.y = float(pos[1])
    t.transform.translation.z = float(pos[2])
    t.transform.rotation.x = float(q[0])
    t.transform.rotation.y = float(q[1])
    t.transform.rotation.z = float(q[2])
    t.transform.rotation.w = float(q[3])
    return t


def make_pose_with_covariance(T: np.ndarray, frame_id: str, stamp,
                              fitness: float = 1.0) -> PoseWithCovarianceStamped:
    """4×4 dönüşüm + fitness → PoseWithCovarianceStamped."""
    pos = T[:3, 3]
    q   = Rotation.from_matrix(T[:3, :3]).as_quat()

    msg = PoseWithCovarianceStamped()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    msg.pose.pose.position.x    = float(pos[0])
    msg.pose.pose.position.y    = float(pos[1])
    msg.pose.pose.position.z    = float(pos[2])
    msg.pose.pose.orientation.x = float(q[0])
    msg.pose.pose.orientation.y = float(q[1])
    msg.pose.pose.orientation.z = float(q[2])
    msg.pose.pose.orientation.w = float(q[3])

    cov = (1.0 - fitness) * 0.1
    msg.pose.covariance[0]  = cov          # x
    msg.pose.covariance[7]  = cov          # y
    msg.pose.covariance[14] = cov          # z
    msg.pose.covariance[35] = cov * 0.1    # yaw
    return msg
