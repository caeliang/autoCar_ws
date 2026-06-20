from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    traffic_light_model_path = LaunchConfiguration("traffic_light_model_path")
    traffic_sign_model_path = LaunchConfiguration("traffic_sign_model_path")
    direction_model_path = LaunchConfiguration("direction_model_path")
    image_topic = LaunchConfiguration("image_topic")
    confidence_threshold = LaunchConfiguration("confidence_threshold")
    annotated_topic = LaunchConfiguration("annotated_topic")
    show_image_view = LaunchConfiguration("show_image_view")
    show_raw_image_view = LaunchConfiguration("show_raw_image_view")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "traffic_light_model_path",
                default_value="/home/ranim/autoCar_ws/isik_modeli.pt",
                description="Traffic light YOLO model path.",
            ),
            DeclareLaunchArgument(
                "traffic_sign_model_path",
                default_value="/home/ranim/autoCar_ws/tabela_modeli (1).pt",
                description="Traffic sign YOLO model path.",
            ),
            DeclareLaunchArgument(
                "direction_model_path",
                default_value="/home/ranim/autoCar_ws/solsag_modeli (1).pt",
                description="Direction YOLO model path.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/prius/front_camera/image_raw",
                description="Gazebo camera image topic.",
            ),
            DeclareLaunchArgument(
                "confidence_threshold",
                default_value="0.35",
                description="Minimum confidence for published detections.",
            ),
            DeclareLaunchArgument(
                "annotated_topic",
                default_value="/perception/yolo/annotated_image",
                description="Annotated image topic published by YOLO node.",
            ),
            DeclareLaunchArgument(
                "show_image_view",
                default_value="true",
                description="Open camera window for annotated detections.",
            ),
            DeclareLaunchArgument(
                "show_raw_image_view",
                default_value="true",
                description="Open extra window for raw camera image.",
            ),
            Node(
                package="perception",
                executable="yolo_gazebo_detector.py",
                name="yolo_gazebo_detector",
                output="screen",
                parameters=[
                    {
                        "traffic_light_model_path": traffic_light_model_path,
                        "traffic_sign_model_path": traffic_sign_model_path,
                        "direction_model_path": direction_model_path,
                        "image_topic": image_topic,
                        "confidence_threshold": confidence_threshold,
                        "annotated_topic": annotated_topic,
                        "publish_annotated_image": True,
                    }
                ],
            ),
            Node(
                package="image_view",
                executable="image_view",
                name="yolo_annotated_view",
                output="screen",
                remappings=[("image", annotated_topic)],
                condition=IfCondition(show_image_view),
            ),
            Node(
                package="image_view",
                executable="image_view",
                name="yolo_raw_view",
                output="screen",
                remappings=[("image", image_topic)],
                condition=IfCondition(show_raw_image_view),
            ),
        ]
    )
