#!/usr/bin/env python3
"""
Phase 2 Minimal Gazebo Launch File
軽量なシミュレーション環境でFirebase Bridgeを起動
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    軽量なGazebo環境 + TurtleBot3 + Firebase Bridge を起動

    特徴:
    - phase2_minimal.world を使用（影なし、障害物最小限）
    - 単一ロボットでの動作確認用
    - Nav2とFirebase Bridgeを同時起動
    """

    # パッケージパスを取得
    pkg_ros2_firebase_bridge = FindPackageShare('ros2_firebase_bridge')
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    # ===== 環境変数設定 =====
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')

    # ===== Worldファイルのパス =====
    world_file = PathJoinSubstitution([
        pkg_ros2_firebase_bridge,
        'worlds',
        'phase2_minimal.world'
    ])

    # ===== Gazebo起動 =====
    gazebo_server = ExecuteProcess(
        cmd=['gzserver',
             '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen'
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # ===== TurtleBot3スポーン =====
    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_waffle',
            '-file', PathJoinSubstitution([
                pkg_turtlebot3_gazebo,
                'models',
                'turtlebot3_waffle',
                'model.sdf'
            ]),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # ===== Nav2起動 =====
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_nav2_bringup,
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': PathJoinSubstitution([
                pkg_ros2_firebase_bridge,
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )

    # ===== Firebase Bridge起動 =====
    firebase_bridge = Node(
        package='ros2_firebase_bridge',
        executable='firebase_bridge',
        name='firebase_bridge',
        parameters=[{
            'robot_id': 'robot_001',
            'robot_namespace': '/turtlebot3'
        }],
        output='screen'
    )

    # ===== RViz起動（オプション） =====
    rviz_config = PathJoinSubstitution([
        pkg_nav2_bringup,
        'rviz',
        'nav2_default_view.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        # Gazebo
        gazebo_server,
        gazebo_client,

        # TurtleBot3
        spawn_turtlebot,

        # Nav2
        nav2_launch,

        # Firebase Bridge
        firebase_bridge,

        # RViz (オプション)
        # rviz_node,  # コメント解除で起動
    ])