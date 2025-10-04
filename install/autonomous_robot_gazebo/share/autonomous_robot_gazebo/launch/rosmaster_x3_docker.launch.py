#!/usr/bin/env python3
"""Gazebo-only launch file - use manual teleop in separate terminal."""

import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for Gazebo simulation only."""
    description_share = FindPackageShare('autonomous_robot_description')
    gazebo_share = FindPackageShare('autonomous_robot_gazebo')

    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    entity = LaunchConfiguration('entity')
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    yaw_pose = LaunchConfiguration('yaw')

    world_path = PathJoinSubstitution([gazebo_share, 'worlds', world])
    models_path = PathJoinSubstitution([gazebo_share, 'models'])

    # Controller configuration file
    controllers_config = PathJoinSubstitution([
        description_share,
        'config',
        'rosmaster_x3',
        'simple_controllers.yaml'
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution([description_share, 'urdf', 'robots', 'rosmaster_x3.urdf.xacro']),
            ' ',
            'robot_name:=', robot_name,
            ' ',
            'prefix:=', prefix,
            ' ',
            'use_gazebo:=true'
        ]),
        value_type=str,
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path, ':', EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')]
    )

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[models_path, ':', EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value='')]
    )

    gz_ros2_control_plugin_dir = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
    set_gz_system_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            gz_ros2_control_plugin_dir,
            ':',
            EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value='')
        ]
    )

    start_gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '3', '-r', world_path],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            },
        ],
    )

    twist_stamper = Node(
        package='autonomous_robot_bringup',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', entity,
                    '-topic', 'robot_description',
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', z_pose,
                    '-Y', yaw_pose,
                ],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )

    load_joint_state_broadcaster = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--param-file', controllers_config],
                output='screen'
            )
        ]
    )

    load_mecanum_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['mecanum_drive_controller', '--param-file', controllers_config],
                output='screen'
            )
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

        set_gz_resource_path,
        set_ign_resource_path,
        set_gz_system_plugin_path,
        start_gz_sim,
        robot_state_publisher,
        twist_stamper,
        spawn_robot,
        load_joint_state_broadcaster,
        load_mecanum_controller,
    ])