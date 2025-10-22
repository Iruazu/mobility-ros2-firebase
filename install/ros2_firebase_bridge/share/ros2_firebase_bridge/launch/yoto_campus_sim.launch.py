#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # パッケージパス
    pkg_ros2_firebase = FindPackageShare('ros2_firebase_bridge')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')

    # TurtleBot3モデル設定
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    # URDFファイルパス
    urdf_file = os.path.join(
        pkg_turtlebot3_description,
        'urdf',
        'turtlebot3_burger.urdf'
    )

    # URDFを読み込み
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # ★★★ World ファイルパス（軽量版）★★★
    world_file = PathJoinSubstitution([
        pkg_ros2_firebase,
        'worlds',
        'yoto_campus_simple.world'  # ★ 変更
    ])

    # Gazebo Server起動（PAUSE状態）
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_gazebo_ros,
            '/launch/gzserver.launch.py'
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
            'physics': 'ode',
            'pause': 'true',  # 初期PAUSE
            'extra_gazebo_args': ''
        }.items()
    )

    # Gazebo Client起動
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_gazebo_ros,
            '/launch/gzclient.launch.py'
        ])
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # TurtleBot3をスポーン（高い位置から）
    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '-5.0',  # 正門の少し南
            '-z', '2.0',   # 高めに配置
            '-Y', '1.57',  # 北向き（90度）
            '-timeout', '120.0'
        ],
        output='screen'
    )

    # 5秒後に物理演算を再開
    unpause_physics = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/unpause_physics', 'std_srvs/srv/Empty'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        joint_state_publisher,
        spawn_turtlebot,
        unpause_physics,
    ])