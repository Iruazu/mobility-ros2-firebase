import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_ros2_firebase_bridge = get_package_share_directory('ros2_firebase_bridge')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world_file = os.path.join(pkg_ros2_firebase_bridge, 'worlds', 'utsunomiya_campus.world')

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(os.path.join(
                pkg_turtlebot3_gazebo,
                'models', 'turtlebot3_waffle', 'model.sdf'
            )).read()
        }]
    )

    # Spawn Robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-file', os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_waffle', 'model.sdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    # SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'mapping'
        }]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        slam_toolbox
    ])
