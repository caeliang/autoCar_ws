from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/prius/front_camera/image_raw',
            description='Input camera topic for lane detector'
        ),
        Node(
            package='perception',
            executable='lane_detector_node',
            name='lane_detector',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'show_debug': True,
                'show_debug_window': False,
                'publish_debug_image': True,
                'debug_image_topic': '/lane/debug_image',
                'image_qos_reliable': True,
                'image_qos_depth': 10,
            }]
        ),
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
