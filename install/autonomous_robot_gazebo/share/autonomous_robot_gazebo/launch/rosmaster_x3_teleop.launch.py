#!/usr/bin/env python3
"""Launch the Rosmaster X3 with keyboard teleop control in Gazebo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for robot with keyboard teleop."""

    # Launch configuration variables
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    entity = LaunchConfiguration('entity')
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    yaw_pose = LaunchConfiguration('yaw')

    # Include the main Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('autonomous_robot_gazebo'),
                'launch',
                'rosmaster_x3_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
            'robot_name': robot_name,
            'entity': entity,
            'prefix': prefix,
            'use_sim_time': use_sim_time,
            'x': x_pose,
            'y': y_pose,
            'z': z_pose,
            'yaw': yaw_pose,
        }.items()
    )

    # Teleop keyboard node
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.world', description='Gazebo world file'),
        DeclareLaunchArgument('robot_name', default_value='rosmaster_x3', description='Robot name'),
        DeclareLaunchArgument('entity', default_value=LaunchConfiguration('robot_name'), description='Entity name in Gazebo'),
        DeclareLaunchArgument('prefix', default_value='', description='Joint name prefix'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('x', default_value='0.0', description='Initial x position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial y position'),
        DeclareLaunchArgument('z', default_value='0.1', description='Initial z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw orientation'),

        gazebo_launch,
        teleop_keyboard,
    ])