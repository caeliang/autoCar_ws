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
                'max_speed': 1.0,
                'min_speed': 0.3,
                'straight_speed': 1.0,
                'turn_speed': 0.7,
                'min_lookahead': 3.0,
                'max_lookahead': 7.0,  
                'lookahead_ratio': 2.0,
                'max_omega': 0.7,
                'speed_filter_alpha': 0.18,
                'curvature_slowdown_gain': 4.0,
                'goal_tolerance': 1.0,
                'parking_brake_on_goal': True,
            }]
        )
    ])
