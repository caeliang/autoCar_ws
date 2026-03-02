"""
Launch: Waypoint kayıt modu

Localization + WaypointRecorder çalıştırır.
Araç sürerken ENTER ile waypoint kaydedersin.

Kullanım:
  ros2 launch path_planning waypoint_record.launch.py
  ros2 launch path_planning waypoint_record.launch.py output_file:=/path/to/waypoints.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Arguments ─────────────────────────────────────────────────────
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='',
        description='Waypoint çıktı dosyası (boşsa otomatik isim)')

    min_dist_arg = DeclareLaunchArgument(
        'min_distance',
        default_value='0.5',
        description='Minimum waypoint arası mesafe (m)')

    # ── Waypoint Recorder (Python) ────────────────────────────────────
    recorder = Node(
        package='path_planning',
        executable='waypoint_recorder.py',
        name='waypoint_recorder',
        output='screen',
        parameters=[{
            'output_file': LaunchConfiguration('output_file'),
            'min_distance': LaunchConfiguration('min_distance'),
        }],
        prefix='xterm -e' if os.environ.get('DISPLAY') else '',
    )

    # ── Waypoint Manager (görselleştirme için) ────────────────────────
    manager = Node(
        package='path_planning',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen',
        parameters=[{
            'waypoint_file': '',
            'frame_id': 'map',
            'goal_tolerance': 1.5,
            'loop': False,
            'publish_rate': 5.0,
        }],
    )

    return LaunchDescription([
        output_file_arg,
        min_dist_arg,
        recorder,
        manager,
    ])
