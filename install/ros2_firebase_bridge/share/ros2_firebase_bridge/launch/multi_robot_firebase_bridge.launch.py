#!/usr/bin/env python3
"""
Phase 2: 複数ロボット対応 Launch ファイル
3台のTurtleBot3を異なるnamespaceで起動
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Phase 2: 3台のロボット + それぞれのFirebase Bridgeを起動

    起動されるノード:
    - robot_001 (/robot1 namespace)
    - robot_002 (/robot2 namespace)
    - robot_003 (/robot3 namespace)
    """

    # パッケージパスを取得
    pkg_ros2_firebase_bridge = FindPackageShare('ros2_firebase_bridge')
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')

    # Gazebo World起動(共通)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot3_gazebo,
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ])
    )

    # ===== Robot 1 (robot_001) =====
    robot1_firebase_bridge = Node(
        package='ros2_firebase_bridge',
        executable='firebase_bridge',
        name='firebase_bridge_robot1',
        namespace='robot1',
        parameters=[{
            'robot_id': 'robot_001',
            'robot_namespace': '/robot1'
        }],
        output='screen'
    )

    # ===== Robot 2 (robot_002) =====
    robot2_firebase_bridge = Node(
        package='ros2_firebase_bridge',
        executable='firebase_bridge',
        name='firebase_bridge_robot2',
        namespace='robot2',
        parameters=[{
            'robot_id': 'robot_002',
            'robot_namespace': '/robot2'
        }],
        output='screen'
    )

    # ===== Robot 3 (robot_003) =====
    robot3_firebase_bridge = Node(
        package='ros2_firebase_bridge',
        executable='firebase_bridge',
        name='firebase_bridge_robot3',
        namespace='robot3',
        parameters=[{
            'robot_id': 'robot_003',
            'robot_namespace': '/robot3'
        }],
        output='screen'
    )

    return LaunchDescription([
        # Gazebo World
        gazebo_launch,

        # Firebase Bridges
        robot1_firebase_bridge,
        robot2_firebase_bridge,
        robot3_firebase_bridge,
    ])