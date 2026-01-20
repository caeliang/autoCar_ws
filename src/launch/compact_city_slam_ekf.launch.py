#!/usr/bin/env python3
"""
Compact City - SLAM + EKF Launch File (3D Lidar)
Runs SLAM Toolbox with EKF sensor fusion for accurate localization and mapping.

Sensors used:
- /prius/scan (3D Lidar - PointCloud2)
- /prius/scan_2d (Converted 2D LaserScan for SLAM)
- /prius/odom (Odometry)
- /prius/imu (IMU)
- /prius/gps/fix (GPS)
- /prius/camera/image_raw (Camera)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get config file
    pkg_dir = get_package_share_directory('localization')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf_compact.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        
        # ==========================================
        # PointCloud to LaserScan Converter
        # Converts 3D PointCloud2 to 2D LaserScan for SLAM
        # ==========================================
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'lidar_link',
                'transform_tolerance': 0.01,
                'min_height': -0.5,
                'max_height': 0.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0175,  # ~1 degree
                'scan_time': 0.05,
                'range_min': 0.2,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            remappings=[
                ('cloud_in', '/prius/scan'),
                ('scan', '/prius/scan_2d')
            ]
        ),
        
        # ==========================================
        # EKF Node - Fuses Odom + IMU
        # ==========================================
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/filtered')
            ]
        ),
        
        # ==========================================
        # NavSat Transform - GPS to Odometry
        # ==========================================
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu', '/prius/imu'),
                ('gps/fix', '/prius/gps/fix'),
                ('odometry/filtered', '/odometry/filtered'),
                ('odometry/gps', '/odometry/gps'),
                ('gps/filtered', '/gps/filtered')
            ]
        ),
        
        # ==========================================
        # SLAM Toolbox - Mapping with 2D LaserScan
        # ==========================================
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                
                # Frames
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'chassis',
                'scan_topic': '/prius/scan_2d',
                
                # Mode
                'mode': 'mapping',
                
                # Map parameters
                'resolution': 0.05,
                'max_laser_range': 25.0,
                'minimum_time_interval': 0.5,
                'transform_timeout': 0.5,
                'tf_buffer_duration': 30.0,
                
                # Scan matching
                'minimum_travel_distance': 0.3,
                'minimum_travel_heading': 0.3,
                'scan_buffer_size': 10,
                'scan_buffer_maximum_scan_distance': 10.0,
                
                # Loop closure
                'do_loop_closing': True,
                'loop_match_minimum_chain_size': 10,
                'loop_search_maximum_distance': 3.0,
                
                # Correlation parameters
                'correlation_search_space_dimension': 0.5,
                'correlation_search_space_resolution': 0.01,
                'correlation_search_space_smear_deviation': 0.1,
                
                # Loop search parameters  
                'loop_search_space_dimension': 8.0,
                'loop_search_space_resolution': 0.05,
                'loop_search_space_smear_deviation': 0.03,
                
                # Matching parameters
                'distance_variance_penalty': 0.5,
                'angle_variance_penalty': 1.0,
                'fine_search_angle_offset': 0.00349,
                'coarse_search_angle_offset': 0.349,
                'coarse_angle_resolution': 0.0349,
                'minimum_angle_penalty': 0.9,
                'minimum_distance_penalty': 0.5,
                'use_response_expansion': True,
            }],
            remappings=[
                ('/scan', '/prius/scan_2d'),
                ('/odom', '/odometry/filtered')
            ]
        ),
        
        # ==========================================
        # Static TF Publishers
        # ==========================================
        
        # chassis -> lidar_link (z=1.0m - center of scan range)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_lidar_tf',
            arguments=['--x', '0', '--y', '0', '--z', '1.0',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'chassis', '--child-frame-id', 'lidar_link']
        ),
        
        # chassis -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_camera_tf',
            arguments=['--x', '0', '--y', '-1.5', '--z', '1.5',
                      '--roll', '0', '--pitch', '0', '--yaw', '-1.5708',
                      '--frame-id', 'chassis', '--child-frame-id', 'camera_link']
        ),
        
        # chassis -> gps_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_gps_tf',
            arguments=['--x', '0', '--y', '0', '--z', '1.5',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'chassis', '--child-frame-id', 'gps_link']
        ),
        
        # chassis -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_imu_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'chassis', '--child-frame-id', 'imu_link']
        ),
    ])
