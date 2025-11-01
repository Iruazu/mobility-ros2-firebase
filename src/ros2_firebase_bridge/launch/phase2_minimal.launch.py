#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import sys

def must_exist(path: str, label: str) -> str:
    if not os.path.exists(path):
        print(f"[FATAL] {label} not found: {path}", file=sys.stderr)
        raise FileNotFoundError(path)
    return path

def generate_launch_description():
    # --- 必須ファイル絶対パス ---
    gazebo_launch_py = must_exist(
        "/opt/ros/humble/share/turtlebot3_gazebo/launch/turtlebot3_world.launch.py",
        "Gazebo launch"
    )
    nav2_launch_py = must_exist(
        "/opt/ros/humble/share/turtlebot3_navigation2/launch/navigation2.launch.py",
        "Nav2 launch"
    )
    map_yaml = must_exist(
        "/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml",
        "Map YAML"
    )
    params_file = must_exist(
        "/opt/ros/humble/share/turtlebot3_navigation2/param/burger.yaml",
        "Nav2 params_file"
    )

    # --- TurtleBot3のモデル固定 ---
    set_tb3_model = SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="burger")

    # --- Gazebo 起動 ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_py)
    )

    # --- Navigation2 起動 ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_py),
        launch_arguments={
            "use_sim_time": "true",
            "map": map_yaml,
            "params_file": params_file
        }.items()
    )

    # --- Firebase Bridge 起動 ---
    firebase_service_account = "/home/obana/mobility-ros2-firebase/config/serviceAccountKey.json"
    must_exist(firebase_service_account, "Firebase Service Account")

    firebase_bridge = TimerAction(
        period=15.0,  # Nav2安定後に起動
        actions=[
            Node(
                package="ros2_firebase_bridge",
                executable="firebase_bridge",
                name="firebase_bridge",
                output="screen",
                parameters=[{
                    "robot_id": "robot_001",
                    "robot_namespace": "/turtlebot3",
                    "service_account_path": firebase_service_account
                }],
                env={
                    "GOOGLE_APPLICATION_CREDENTIALS": firebase_service_account
                }
            )
        ]
    )

    return LaunchDescription([
        set_tb3_model,
        gazebo_launch,
        nav2_launch,
        firebase_bridge,
    ])
