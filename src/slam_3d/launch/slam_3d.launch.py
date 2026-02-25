#!/usr/bin/env python3
"""
3D SLAM Launch File with Octomap
Creates 3D point cloud maps using the Octomap server
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get config file paths
    slam_3d_dir = get_package_share_directory('slam_3d')
    localization_dir = get_package_share_directory('localization')
    
    octomap_config = os.path.join(slam_3d_dir, 'config', 'octomap_params.yaml')
    ekf_config = os.path.join(localization_dir, 'config', 'ekf_compact.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
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
        # Octomap Server - 3D Mapping
        # Creates 3D voxel-based maps from PointCloud2
        # ==========================================
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[octomap_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cloud_in', '/prius/scan')
            ]
        ),
        
        # ==========================================
        # SLAM Toolbox - 2D SLAM for comparison
        # (Optional - can be removed if only 3D needed)
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
                'map_update_interval': 1.0,
                
                # Scan matching
                'minimum_travel_distance': 0.2,
                'minimum_travel_heading': 0.2,
                'scan_buffer_size': 20,
                'scan_buffer_maximum_scan_distance': 15.0,
                
                # Correlation
                'correlation_search_space_dimension': 0.5,
                'correlation_search_space_resolution': 0.01,
                'correlation_search_space_smear_deviation': 0.1,
                
                # Loop closure
                'loop_search_maximum_distance': 3.0,
                'do_loop_closing': True,
                'loop_match_minimum_chain_size': 10,
                'loop_match_maximum_variance_coarse': 3.0,
                'loop_match_minimum_response_coarse': 0.35,
                
                # Scan matcher
                'distance_variance_penalty': 0.5,
                'angle_variance_penalty': 1.0,
                'fine_search_angle_offset': 0.00349,
                'coarse_search_angle_offset': 0.349,
                'coarse_angle_resolution': 0.0349,
                'minimum_angle_penalty': 0.9,
                'minimum_distance_penalty': 0.5,
                'use_response_expansion': True,
            }]
        ),
        
        # ==========================================
        # PointCloud to LaserScan Converter
        # For 2D SLAM compatibility
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
                'angle_increment': 0.0175,
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
        # Static TF Publishers
        # Required for proper frame transforms
        # ==========================================
        
        # chassis -> lidar_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_lidar_tf',
            arguments=['--x', '0', '--y', '0', '--z', '1.0',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'chassis', '--child-frame-id', 'lidar_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # chassis -> sensor_link  (Gazebo LiDAR frame_name = sensor_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_sensor_link_tf',
            arguments=['--x', '0', '--y', '0', '--z', '1.0',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'chassis', '--child-frame-id', 'sensor_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # chassis -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_camera_tf',
            arguments=['--x', '0', '--y', '-1.5', '--z', '1.5',
                      '--roll', '0', '--pitch', '0', '--yaw', '-1.5708',
                      '--frame-id', 'chassis', '--child-frame-id', 'camera_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # chassis -> gps_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_gps_tf',
            arguments=['--x', '0', '--y', '0', '--z', '1.5',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'chassis', '--child-frame-id', 'gps_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # chassis -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_imu_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'chassis', '--child-frame-id', 'imu_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
