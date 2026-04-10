from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen',
            parameters=[{
                'max_speed': 2.0,
                'min_speed': 0.3,
                'min_lookahead': 1.5,
                'max_lookahead': 5.0,
                'lookahead_ratio': 1.5,
                'max_omega': 1.0,
            }]
        )
    ])
