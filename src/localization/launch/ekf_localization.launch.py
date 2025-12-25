#!/usr/bin/env python3
"""
EKF Localization Launch File
Fuses: Odometry (/prius/odom), IMU (/prius/imu), GPS (/prius/gps/fix)

Architecture:
  1. EKF Local (odom frame): Odom + IMU -> /odometry/filtered/local
  2. NavSat Transform: GPS -> /odometry/gps
  3. EKF Global (map frame): Local EKF + GPS + IMU -> /odometry/filtered/global
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get config file path
    pkg_dir = get_package_share_directory('localization')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # EKF Local - Fuses Odometry + IMU (odom frame)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/filtered/local')
            ]
        ),
        
        # NavSat Transform - Converts GPS to odometry
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu', '/prius/imu'),
                ('gps/fix', '/prius/gps/fix'),
                ('odometry/filtered', '/odometry/filtered/local'),
                ('odometry/gps', '/odometry/gps'),
                ('gps/filtered', '/gps/filtered')
            ]
        ),
        
        # EKF Global - Fuses Local EKF + GPS (map frame)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/filtered/global')
            ]
        ),
        
        # Static transform: map -> odom (initial)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        
        # Static transform: chassis -> gps_link (GPS sensor position on vehicle)
        # GPS is typically mounted on the roof, ~1.5m above chassis
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_gps_tf',
            arguments=['0', '0', '1.5', '0', '0', '0', 'chassis', 'gps_link']
        ),
        
        # Static transform: chassis -> imu_link (IMU sensor position)
        # IMU is typically at the center of the vehicle
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_imu_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'chassis', 'imu_link']
        ),
    ])
