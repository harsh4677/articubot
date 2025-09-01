# /home/harsh/dev_ws/src/articubot/launch/launch_sim.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'articubot'

    # World Argument
    default_world = os.path.join(get_package_share_directory(package_name), 'worlds', 'articubot_world.sdf')
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to the world file to load'
    )

    # RSP Launch Include
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Gazebo Launch Include
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Node to spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.1'],
        output='screen'
    )
    
    # Controller Spawners
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Gazebo <-> ROS Bridge
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # --- CORRECTED: Simplified Launch Logic ---
    # Use a timer to delay the robot spawn, giving Gazebo time to load.
    # This is more robust than chaining events off an IncludeLaunchDescription.
    delayed_spawn_and_bridge = TimerAction(
        period=5.0,
        actions=[spawn_entity, ros_gz_bridge]
    )

    # Chain the controller spawners to run after the robot is spawned.
    # This ensures the /controller_manager service is available.
    spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
    )

    load_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    return LaunchDescription([
        rsp,
        world_arg,
        gazebo,
        delayed_spawn_and_bridge,
        spawn_controllers,
        load_diff_drive_controller
    ])