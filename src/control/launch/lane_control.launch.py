from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            executable='lane_controller',
            name='lane_controller',
            output='screen',
            parameters=[{
                'target_speed': 0.38,
                'max_angular': 0.22,
                'kp': 0.18,
                'ki': 0.004,
                'kd': 0.014,
                'heading_gain': 0.05,
                'error_alpha': 0.08,
                'steer_sign': -1.0,
                'min_speed_factor': 1.0,
                'omega_speed_gain': 0.0,
                'speed_sign': 1.0,
                'log_control': True,
            }]
        )
    ])
