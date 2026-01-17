"""
Localization Launch File for AutoCar
Uses AMCL with a saved map for localization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Get package directories
    slam_mapping_dir = get_package_share_directory('slam_mapping')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Default map path
    default_map = os.path.join(slam_mapping_dir, 'maps', 'city_map.yaml')
    
    # AMCL parameters file
    amcl_params_file = os.path.join(
        slam_mapping_dir, 'config', 'amcl_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),
        
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file'),
        
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'),
        
        # Static transform: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'laser'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_file}
            ],
        ),
        
        # AMCL Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                amcl_params_file,
                {'use_sim_time': use_sim_time}
            ],
        ),
        
        # Lifecycle Manager for map_server and amcl
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': ['map_server', 'amcl']}
            ],
        ),
    ])
