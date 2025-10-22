#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():

    # パッケージパス
    pkg_ros2_firebase = FindPackageShare('ros2_firebase_bridge')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    # マップファイルパス
    map_file = PathJoinSubstitution([
        pkg_ros2_firebase,
        'maps',
        'yoto_campus_map.yaml'
    ])

    # Nav2パラメータファイル
    params_file = PathJoinSubstitution([
        pkg_ros2_firebase,
        'config',
        'nav2_params_yoto.yaml'
    ])

    # Gazebo + TurtleBot3起動
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_ros2_firebase,
            '/launch/yoto_campus_sim.launch.py'
        ])
    )

    # Nav2起動（10秒遅延）
    nav2_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    pkg_nav2_bringup,
                    '/launch/bringup_launch.py'
                ]),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': 'true',
                    'params_file': params_file,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # Firebase Bridge起動（15秒遅延）
    firebase_bridge = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='ros2_firebase_bridge',
                executable='firebase_bridge',
                name='firebase_bridge',
                output='screen',
                parameters=[{
                    'mode': 'simulation',
                    'origin_lat': 36.55077,
                    'origin_lng': 139.92957,
                    'scale_factor': 1.0,
                    'map_bounds': {
                        'x_min': -50.0,  # ★ 100m範囲
                        'x_max': 50.0,
                        'y_min': -50.0,
                        'y_max': 50.0
                    }
                }]
            )
        ]
    )

    # RViz起動（12秒遅延）
    rviz = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', PathJoinSubstitution([
                    pkg_nav2_bringup,
                    'rviz',
                    'nav2_default_view.rviz'
                ])],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        nav2_launch,
        firebase_bridge,
        rviz,
    ])