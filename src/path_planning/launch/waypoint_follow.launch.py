"""
Launch: Waypoint takip modu v5

WaypointManager (segment-aware) + PurePursuit (path-based)

CSV dosyasından tüm waypointleri yükler, otomatik segmentlere
ayırır ve sırayla takip eder. Kavşaklarda segment geçişi yapar.

Kullanım:
  ros2 launch path_planning waypoint_follow.launch.py \
      waypoint_file:=/path/to/waypoints.csv
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('path_planning')
    default_params = os.path.join(pkg_share, 'config', 'waypoint_params.yaml')

    # ── Arguments ─────────────────────────────────────────────────────
    wp_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='',
        description='Waypoint CSV dosyası yolu')

    loop_arg = DeclareLaunchArgument(
        'loop', default_value='true',
        description='Segment döngüsü (true/false)')

    max_speed_arg = DeclareLaunchArgument(
        'max_speed', default_value='2.0',
        description='Maksimum hız (m/s)')

    # ── Waypoint Manager (C++ — segment-aware) ───────────────────────
    manager = Node(
        package='path_planning',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen',
        parameters=[
            default_params,
            {
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'loop': LaunchConfiguration('loop'),
            }
        ],
    )

    # ── Pure Pursuit (C++ — path-based) ──────────────────────────────
    pursuit = Node(
        package='path_planning',
        executable='pure_pursuit',
        name='pure_pursuit',
        output='screen',
        parameters=[
            default_params,
            {
                'max_speed': LaunchConfiguration('max_speed'),
            }
        ],
    )

    return LaunchDescription([
        wp_file_arg,
        loop_arg,
        max_speed_arg,
        manager,
        pursuit,
    ])
