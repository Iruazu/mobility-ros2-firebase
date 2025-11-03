import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # パッケージディレクトリ
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_ros2_firebase_bridge = get_package_share_directory('ros2_firebase_bridge')

    # Worldファイルのパス
    world_file = os.path.join(pkg_ros2_firebase_bridge, 'worlds', 'utsunomiya_campus.world')
    # Gazebo起動
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Robot 1
    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_robot1',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models', 'turtlebot3_waffle', 'model.sdf'
            )).read()
        }]
    )

    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot1',
            '-file', os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models', 'turtlebot3_waffle', 'model.sdf'
            ),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-robot_namespace', 'robot1'
        ],
        output='screen'
    ) 
    # Robot 2
    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_robot2',
        namespace='robot2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models', 'turtlebot3_waffle', 'model.sdf'
            )).read()
        }]
    )

    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot2',
            '-file', os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models', 'turtlebot3_waffle', 'model.sdf'
            ),
            '-x', '2.0',
            '-y', '2.0',
            '-z', '0.01',
            '-robot_namespace', 'robot2'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot1_state_publisher,
        robot2_state_publisher,
        spawn_robot1,
        spawn_robot2
    ])
