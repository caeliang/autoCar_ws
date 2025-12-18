#!/usr/bin/env python3
"""
ROS2 Launch file for AutoCar Gazebo Simulation
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    # Get workspace path
    workspace_dir = os.path.expanduser('~/autoCar_ws')
    
    # Plugin path
    plugin_path = os.path.join(workspace_dir, 'install', 'gazebo_traffic_light_plugin', 'lib')
    
    # World file path
    world_file = os.path.join(workspace_dir, 'src', 'simple_city_copy.world')
    
    # Current GAZEBO_PLUGIN_PATH
    current_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    new_plugin_path = f"{plugin_path}:{current_plugin_path}" if current_plugin_path else plugin_path
    
    return LaunchDescription([
        # Set environment variables
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', new_plugin_path),
        
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen',
            env={
                'GAZEBO_PLUGIN_PATH': new_plugin_path,
                'GAZEBO_MODEL_PATH': os.path.expanduser('~/.gazebo/models')
            }
        ),
    ])
