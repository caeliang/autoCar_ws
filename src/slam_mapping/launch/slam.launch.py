"""
SLAM Launch File for AutoCar
Uses slam_toolbox for online SLAM mapping
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    slam_mapping_dir = get_package_share_directory('slam_mapping')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # SLAM Toolbox parameters file
    slam_params_file = os.path.join(
        slam_mapping_dir, 'config', 'slam_toolbox_params.yaml')
    
    # Path to odom_to_tf script
    odom_to_tf_script = '/home/ranim/autoCar_ws/scripts/odom_to_tf.py'
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),
        
        # Static transform: chassis -> center_laser (LiDAR frame from Prius)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_laser_tf',
            arguments=['0', '0', '1.2', '0', '0', '0', 'chassis', 'center_laser'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Odom to TF broadcaster (publishes odom -> chassis from /prius/odom)
        ExecuteProcess(
            cmd=['python3', odom_to_tf_script],
            name='odom_to_tf',
            output='screen'
        ),
        
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
    ])
