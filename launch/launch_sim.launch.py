import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for spawning the robot in Gazebo.
    """

    package_name = 'articubot'

    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'articubot_world.sdf')

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + world_path}.items(),
    )

    # Run the spawner node
    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                '-name', 'my_bot',
                                '-z', '0.1'],
                        output='screen')

    # Bridge ROS 2 /cmd_vel messages to Gazebo /cmd_vel messages
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    # NEW BRIDGE FOR ODOMETRY AND TF
    odom_tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
        remappings=[('/odom', 'odom'), ('/tf', 'tf')],
        output='screen'
    )

    laser_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    delayed_actions = TimerAction(
        period=5.0, 
        actions=[
            spawn_entity,
            cmd_vel_bridge,
            odom_tf_bridge,
            laser_bridge
        ]
    )

    # Launch everything
    return LaunchDescription([
        rsp,
        gz_sim,
        delayed_actions,
    ])
