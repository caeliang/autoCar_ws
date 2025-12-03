#!/usr/bin/env python3
"""
Read a ROS2 bag (rosbag2) and plot lidar pose, imu data, and fused pose.

Usage:
  python3 bag_to_plot.py --bag /path/to/bag --out /path/to/output.svg

Defaults:
  lidar topic: /lidar_pose
  imu topic: /imu/data
  fused topic: /fused_pose

Notes:
- Requires ROS 2 python packages: rosbag2_py, rosidl_runtime_py, rclpy (for serialization utilities), and rclpy.serialization
- The script is resilient to message types: supports nav_msgs/msg/Odometry, geometry_msgs/msg/PoseStamped, geometry_msgs/msg/Pose for pose topics.
- IMU: plots linear_acceleration.x/y and yaw (converted from orientation quaternion) over time.
"""

import argparse
import os
import sys
import math
from collections import defaultdict

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message
except Exception as e:
    print("Error importing ROS2 bag libraries:", e)
    print("Make sure this script runs in a ROS2 environment where 'rosbag2_py', 'rclpy', and 'rosidl_runtime_py' are available.")
    sys.exit(2)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def quat_to_yaw(qx, qy, qz, qw):
    # yaw (z-axis rotation) from quaternion
    # Reference: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    ysqr = qy * qy
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (ysqr + qz * qz)
    yaw = math.atan2(t3, t4)
    return yaw


def extract_pose_from_msg(msg, msg_type):
    # Return (x,y) if msg contains pose info, else None
    # Handle nav_msgs/msg/Odometry
    t = msg_type
    try:
        if t.endswith('nav_msgs/msg/Odometry') or msg.__class__.__name__ == 'Odometry':
            p = msg.pose.pose.position
            return float(p.x), float(p.y)
        # geometry_msgs/msg/PoseStamped
        if t.endswith('geometry_msgs/msg/PoseStamped') or msg.__class__.__name__ == 'PoseStamped':
            p = msg.pose.position
            return float(p.x), float(p.y)
        # geometry_msgs/msg/Pose
        if t.endswith('geometry_msgs/msg/Pose') or msg.__class__.__name__ == 'Pose':
            p = msg.position
            return float(p.x), float(p.y)
    except Exception:
        pass
    return None


def extract_imu_from_msg(msg, msg_type):
    # Return (ax, ay, yaw) where ax/ay are linear_acceleration, yaw from orientation
    try:
        la = msg.linear_acceleration
        ax = float(la.x)
        ay = float(la.y)
        q = msg.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        return ax, ay, yaw
    except Exception:
        return None


def read_bag(bag_path, topics):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # map topic name -> type
    topic_types = {}
    for meta in reader.get_all_topics_and_types():
        topic_types[meta.name] = meta.type

    # prepare containers
    data = defaultdict(lambda: {'t': [], 'x': [], 'y': [], 'raw': []})
    imu_data = {'t': [], 'ax': [], 'ay': [], 'yaw': []}

    # Iterate messages
    while reader.has_next():
        (topic, data_blob, t) = reader.read_next()
        # time in nanoseconds
        time_sec = t / 1e9
        if topic not in topic_types:
            continue
        msg_type = topic_types[topic]
        try:
            msg_cls = get_message(msg_type)
            msg = deserialize_message(data_blob, msg_cls)
        except Exception:
            # couldn't deserialize: skip
            continue

        if topic == topics.lidar:
            pose = extract_pose_from_msg(msg, msg_type)
            if pose:
                x, y = pose
                data['lidar']['t'].append(time_sec)
                data['lidar']['x'].append(x)
                data['lidar']['y'].append(y)
                data['lidar']['raw'].append(msg)
        elif topic == topics.fused:
            pose = extract_pose_from_msg(msg, msg_type)
            if pose:
                x, y = pose
                data['fused']['t'].append(time_sec)
                data['fused']['x'].append(x)
                data['fused']['y'].append(y)
                data['fused']['raw'].append(msg)
        elif topic == topics.imu:
            imu = extract_imu_from_msg(msg, msg_type)
            if imu:
                ax, ay, yaw = imu
                imu_data['t'].append(time_sec)
                imu_data['ax'].append(ax)
                imu_data['ay'].append(ay)
                imu_data['yaw'].append(yaw)

    return data, imu_data


def make_plots(data, imu_data, out_path, width=8, height=6, dpi=200):
    fig = plt.figure(figsize=(width, height), dpi=dpi)

    # Main trajectory subplot
    ax1 = fig.add_subplot(2, 1, 1)
    ax1.set_title('Trajectory: LIDAR (green) vs FUSED (red)')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.grid(True)

    lidar_exists = len(data['lidar']['x']) > 0
    fused_exists = len(data['fused']['x']) > 0

    if lidar_exists:
        ax1.plot(data['lidar']['x'], data['lidar']['y'], '-g', label='lidar')
        ax1.scatter(data['lidar']['x'][:1], data['lidar']['y'][:1], c='green', marker='o', label='lidar start')
    if fused_exists:
        ax1.plot(data['fused']['x'], data['fused']['y'], '-r', label='fused')
        ax1.scatter(data['fused']['x'][:1], data['fused']['y'][:1], c='red', marker='x', label='fused start')

    ax1.legend()
    ax1.axis('equal')

    # IMU subplot: accel x,y and yaw
    ax2 = fig.add_subplot(2, 1, 2)
    ax2.set_title('IMU linear_acceleration (x,y) and yaw')
    if len(imu_data['t']) > 0:
        t0 = imu_data['t'][0]
        t_rel = [tt - t0 for tt in imu_data['t']]
        ax2.plot(t_rel, imu_data['ax'], '-b', label='ax')
        ax2.plot(t_rel, imu_data['ay'], '-c', label='ay')
        ax2_twin = ax2.twinx()
        ax2_twin.plot(t_rel, imu_data['yaw'], '-m', label='yaw')
        ax2.set_xlabel('time (s)')
        ax2.set_ylabel('accel (m/s^2)')
        ax2_twin.set_ylabel('yaw (rad)')
        lines, labels = ax2.get_legend_handles_labels()
        lines2, labels2 = ax2_twin.get_legend_handles_labels()
        ax2.legend(lines + lines2, labels + labels2)
    else:
        ax2.text(0.5, 0.5, 'No IMU data found', horizontalalignment='center')

    plt.tight_layout()
    dirname = os.path.dirname(out_path)
    if dirname and not os.path.exists(dirname):
        os.makedirs(dirname, exist_ok=True)
    fig.savefig(out_path)
    print(f'Wrote plot to: {out_path}')


def main():
    p = argparse.ArgumentParser(description='Read ros2 bag and plot lidar/imu/fused topics')
    p.add_argument('--bag', required=True, help='Path to ros2 bag directory (sqlite3 storage)')
    p.add_argument('--out', default=os.path.expanduser('~/autoCar_ws/src/sensor_fusion_pkg/log/fusion_plot_from_bag.svg'), help='Output image path')
    p.add_argument('--lidar', default='/lidar_pose', help='LIDAR topic')
    p.add_argument('--imu', default='/imu/data', help='IMU topic')
    p.add_argument('--fused', default='/fused_pose', help='Fused pose topic')
    p.add_argument('--width', type=float, default=10.0, help='Figure width (inches)')
    p.add_argument('--height', type=float, default=8.0, help='Figure height (inches)')
    p.add_argument('--dpi', type=int, default=200, help='Figure DPI')

    args = p.parse_args()

    class T:
        pass
    topics = T()
    topics.lidar = args.lidar
    topics.imu = args.imu
    topics.fused = args.fused

    if not os.path.exists(args.bag):
        print('Bag path does not exist:', args.bag)
        sys.exit(1)

    print('Reading bag:', args.bag)
    data, imu_data = read_bag(args.bag, topics)
    print('Found samples: lidar={}, fused={}, imu={}'.format(len(data['lidar']['t']), len(data['fused']['t']), len(imu_data['t'])))

    make_plots(data, imu_data, os.path.expanduser(args.out), width=args.width, height=args.height, dpi=args.dpi)


if __name__ == '__main__':
    main()
