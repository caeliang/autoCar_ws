#!/usr/bin/env python3
"""
🚀 Complete Autonomous System Launch
Compact City World with A* Path Planning + Pure Pursuit Control

Hedef Noktalar:
- Start: (10.0, 10.0, 0°)
- Goal: (40.0, 40.0)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    workspace_dir = os.path.expanduser("~/autoCar_ws")

    # ════════════════════════════════════════════════════════════════
    # Environment Setup
    # ════════════════════════════════════════════════════════════════
    plugin_path = os.path.join(
        workspace_dir, "install", "gazebo_traffic_light_plugin", "lib"
    )
    world_file = os.path.join(
        workspace_dir, "src", "worlds", "compact_city.world"  # ← COMPACT CITY
    )
    models_path = os.path.join(workspace_dir, "src", "models")

    current_plugin_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    new_plugin_path = (
        f"{plugin_path}:{current_plugin_path}" if current_plugin_path else plugin_path
    )

    current_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    new_model_path = (
        f"{models_path}:{os.path.expanduser('~/.gazebo/models')}:{current_model_path}"
    )

    # ════════════════════════════════════════════════════════════════
    # Launch Arguments
    # ════════════════════════════════════════════════════════════════
    start_x = DeclareLaunchArgument(
        "start_x", default_value="10.0", description="Start X coordinate"
    )
    start_y = DeclareLaunchArgument(
        "start_y", default_value="10.0", description="Start Y coordinate"
    )
    goal_x = DeclareLaunchArgument(
        "goal_x", default_value="40.0", description="Goal X coordinate"
    )
    goal_y = DeclareLaunchArgument(
        "goal_y", default_value="40.0", description="Goal Y coordinate"
    )

    max_speed_arg = DeclareLaunchArgument(
        "max_speed", default_value="2.0", description="Maximum speed (m/s)"
    )
    max_omega_arg = DeclareLaunchArgument(
        "max_omega", default_value="1.0", description="Maximum angular velocity (rad/s)"
    )

    return LaunchDescription(
        [
            # ───────────────────────────────────────────────────────────
            # Arguments
            # ───────────────────────────────────────────────────────────
            start_x,
            start_y,
            goal_x,
            goal_y,
            max_speed_arg,
            max_omega_arg,
            # ───────────────────────────────────────────────────────────
            # Environment Variables
            # ───────────────────────────────────────────────────────────
            SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", new_plugin_path),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", new_model_path),
            # ───────────────────────────────────────────────────────────
            # 🎮 GAZEBO SIMULATOR
            # ───────────────────────────────────────────────────────────
            ExecuteProcess(
                cmd=["gazebo", "--verbose", world_file],
                name="gazebo_simulator",
                output="screen",
                env={
                    "GAZEBO_PLUGIN_PATH": new_plugin_path,
                    "GAZEBO_MODEL_PATH": new_model_path,
                },
            ),
            # ───────────────────────────────────────────────────────────
            # 📍 LOCALIZATION (ICP-based)
            # ───────────────────────────────────────────────────────────
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    f"source {workspace_dir}/install/setup.bash && "
                    f"ros2 launch localization simple_localization.launch.py "
                    f"map_pcd_path:={workspace_dir}/maps/scaled_build.pcd",
                ],
                name="localization_icp",
                output="screen",
            ),
            # ───────────────────────────────────────────────────────────
            # 🗺️ PATH PLANNING + PURE PURSUIT (Full Autonomous System)
            # ───────────────────────────────────────────────────────────
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    f"source {workspace_dir}/install/setup.bash && "
                    f"ros2 launch path_planning full_autonomous_system.launch.py "
                    f"waypoint_file:={workspace_dir}/waypoints/full_road_map.csv",
                ],
                name="autonomous_system",
                output="screen",
            ),
        ]
    )
