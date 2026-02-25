#!/usr/bin/env python3
"""
Simple Localization Launch File
Basit ve çalışan PCD harita tabanlı lokalizasyon
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch argümanları
    map_pcd_path = LaunchConfiguration('map_pcd_path')
    sensor_z_offset = LaunchConfiguration('sensor_z_offset')
    odom_topic = LaunchConfiguration('odom_topic')
    
    return LaunchDescription([
        # Harita dosya yolu argümanı
        DeclareLaunchArgument(
            'map_pcd_path',
            default_value='',
            description='Path to PCD map file'
        ),
        
        # Sensör yükseklik ofseti (chassis → lidar arası z mesafesi)
        DeclareLaunchArgument(
            'sensor_z_offset',
            default_value='1.0',
            description='Sensor height above chassis (z offset in meters)'
        ),
        
        # Odometri topic'i
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/prius/odom',
            description='Odometry topic for initial pose guess'
        ),
        
        # Simple localizer node
        Node(
            package='localization',
            executable='simple_localizer.py',
            name='simple_localizer',
            output='screen',
            parameters=[{
                'map_pcd_path': map_pcd_path,
                'sensor_z_offset': sensor_z_offset,
                'odom_topic': odom_topic,
                'max_icp_dist': 5.0,
                'icp_iterations': 80,
                'icp_tolerance': 0.001,
            }]
        ),
    ])
