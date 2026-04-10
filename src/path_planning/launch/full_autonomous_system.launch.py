"""
Full autonomous system launch - A* path planning + Pure Pursuit control

Topics architecture:
  /waypoints/path ← path_planning generates (A*)
                 → control subscribes (pure_pursuit)
  
  /prius/odom ← Gazebo publishes
             → control subscribes (pure_pursuit)
  
  /localization/pose ← ICP publishes
                    → control subscribes (pure_pursuit)
  
  /prius/cmd_vel ← control publishes (pure_pursuit)
                → Gazebo subscribes
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Path planning config
    pp_share = get_package_share_directory('path_planning')
    pp_params = os.path.join(pp_share, 'config', 'waypoint_params.yaml')

    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='',
        description='Path to waypoint CSV file'
    )

    # ═══════════════════════════════════════════════════════════════
    # PATH PLANNING NODES
    # ═══════════════════════════════════════════════════════════════

    waypoint_manager = Node(
        package='path_planning',
        executable='waypoint_manager',
        name='waypoint_manager',
        parameters=[pp_params, {'waypoint_file': LaunchConfiguration('waypoint_file')}],
        output='screen',
        emulate_tty=True,
    )

    # ═══════════════════════════════════════════════════════════════
    # CONTROL NODES
    # ═══════════════════════════════════════════════════════════════

    pure_pursuit = Node(
        package='control',
        executable='pure_pursuit',
        name='pure_pursuit',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_speed': 2.0,
            'min_speed': 0.3,
            'min_lookahead': 1.5,
            'max_lookahead': 5.0,
            'lookahead_ratio': 1.5,
            'max_omega': 1.0,
        }]
    )

    return LaunchDescription([
        waypoint_file_arg,
        # Path planning pipeline
        waypoint_manager,
        # Control loop
        pure_pursuit,
    ])
