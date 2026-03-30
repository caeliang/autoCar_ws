import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('path_planning')
    params = os.path.join(pkg_share, 'config', 'waypoint_params.yaml')

    waypoint_file_arg = DeclareLaunchArgument('waypoint_file', default_value='')

    manager = Node(
        package='path_planning',
        executable='waypoint_manager',
        name='waypoint_manager',
        parameters=[params, {'waypoint_file': LaunchConfiguration('waypoint_file')}],
        output='screen',
    )

    follower = Node(
        package='path_planning',
        executable='lane_path_follower',
        name='lane_path_follower',
        parameters=[params],
        output='screen',
    )

    pursuit = Node(
        package='path_planning',
        executable='pure_pursuit',
        name='pure_pursuit',
        parameters=[params],
        output='screen',
    )

    return LaunchDescription([waypoint_file_arg, manager, follower, pursuit])
