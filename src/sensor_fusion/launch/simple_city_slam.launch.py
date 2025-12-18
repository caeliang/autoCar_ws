#!/usr/bin/env python3
"""
Launch Gazebo with the simple_city_copy.world and run SLAM.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the world file
    world_file = os.path.expanduser('~/autoCar_ws/src/simple_city_copy.world')

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )

    # SLAM node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.expanduser('~/autoCar_ws/src/sensor_fusion/config/slam_toolbox_async.yaml')
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/odom', '/odom')
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        slam_toolbox_node
    ])