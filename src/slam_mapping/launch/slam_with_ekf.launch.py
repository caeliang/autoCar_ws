#!/usr/bin/env python3
"""
Integrated SLAM + EKF Launch File for AutoCar
Runs SLAM for mapping while also running EKF for state estimation
Both work together without TF conflicts
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    slam_mapping_dir = get_package_share_directory('slam_mapping')
    localization_dir = get_package_share_directory('localization')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Config files
    slam_params_file = os.path.join(
        slam_mapping_dir, 'config', 'slam_toolbox_params.yaml')
    ekf_config_file = os.path.join(
        localization_dir, 'config', 'ekf.yaml')
    
    # Path to odom_to_tf script
    odom_to_tf_script = os.path.join(
        slam_mapping_dir, 'scripts', 'odom_to_tf.py')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),
        
        # ==================== STATIC TRANSFORMS ====================
        # Static transform: chassis -> center_laser (LiDAR frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_laser_tf',
            arguments=['0', '0', '1.2', '0', '0', '0', 'chassis', 'center_laser'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Static transform: chassis -> gps_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_gps_tf',
            arguments=['0', '0', '1.5', '0', '0', '0', 'chassis', 'gps_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Static transform: chassis -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_imu_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'chassis', 'imu_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Static transform: map -> odom (initial, will be updated by SLAM)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # ==================== ODOMETRY TO TF ====================
        # Odom to TF broadcaster (publishes odom -> chassis from /prius/odom)
        ExecuteProcess(
            cmd=['python3', odom_to_tf_script],
            name='odom_to_tf',
            output='screen'
        ),
        
        # ==================== SLAM TOOLBOX ====================
        # SLAM Toolbox Node (Online Async mode)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
        ),
        
        # ==================== EKF LOCALIZATION ====================
        # EKF Local - Fuses Odometry + IMU (odom frame)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
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
            parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
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
            parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/filtered/global')
            ]
        ),
    ])

