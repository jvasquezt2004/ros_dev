#!/usr/bin/env python3
"""
Launch RViz visualization for the Yahboom (ROSMASTER) robot.

This launch file sets up the complete visualization environment for the robot,
including robot state publisher, joint state publisher, and RViz2. It handles loading
and processing of URDF/XACRO files and controller configurations.

:author: Addison Sears-Collins
:date: November 20, 2024
"""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.logging import get_logger
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
DESCRIPTION_PACKAGE = 'autonomous_robot_description'
LOGGER = get_logger(__name__)


def process_ros2_controllers_config(context):
    """Process the ROS 2 controller configuration yaml file before loading the URDF.

    This function reads a template configuration file, replaces placeholder values
    with actual configuration, and writes the processed file to both source and
    install directories.

    Args:
        context: Launch context containing configuration values

    Returns:
        list: Empty list as required by OpaqueFunction
    """

    # Get the configuration values
    prefix = LaunchConfiguration('prefix').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    enable_odom_tf = LaunchConfiguration('enable_odom_tf').perform(context)

    package_share_path = Path(get_package_share_directory(DESCRIPTION_PACKAGE))
    launch_file_root = Path(__file__).resolve().parent.parent

    config_roots = {
        package_share_path / 'config' / robot_name,
        launch_file_root / 'config' / robot_name,
    }

    template_path = None
    for root in config_roots:
        candidate = root / 'ros2_controllers_template.yaml'
        if candidate.exists():
            template_path = candidate
            break

    if template_path is None:
        return []

    with open(template_path, 'r', encoding='utf-8') as file:
        template_content = file.read()

    # Create processed content (leaving template untouched)
    processed_content = template_content.replace('${prefix}', prefix)
    processed_content = processed_content.replace(
        'enable_odom_tf: true', f'enable_odom_tf: {enable_odom_tf}')

    # Write processed content to both source and install directories
    for config_path in config_roots:
        os.makedirs(config_path, exist_ok=True)
        output_path = config_path / 'ros2_controllers.yaml'
        with open(output_path, 'w', encoding='utf-8') as file:
            file.write(processed_content)

    return []


# Define the arguments for the XACRO file
ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='rosmaster_x3',
                          description='Name of the robot'),
    DeclareLaunchArgument('prefix', default_value='',
                          description='Prefix for robot joints and links'),
    DeclareLaunchArgument('use_gazebo', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to use Gazebo simulation'),
    DeclareLaunchArgument('enable_odom_tf', default_value='true',
                          choices=['true', 'false'],
                          description='Enable odometry transform broadcasting via ROS 2 Control')
]


def configure_joint_state_publishers(context, *args, **kwargs):
    """Create the appropriate joint state publisher node(s).

    Ensures that only one of the GUI/non-GUI publishers runs at a time so the
    GUI sliders can drive the transforms without being overridden by the
    headless publisher.
    """

    use_jsp_value = LaunchConfiguration('use_jsp').perform(context).lower()
    jsp_gui_value = LaunchConfiguration('jsp_gui').perform(context).lower()
    use_sim_time_value = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    actions = []

    if jsp_gui_value == 'true':
        actions.append(Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_sim_time': use_sim_time_value}]
        ))

        if use_jsp_value == 'true':
            LOGGER.warning(
                'Ignoring headless joint_state_publisher because the GUI is enabled; '
                'set jsp_gui:=false if you prefer the non-GUI publisher.'
            )
    elif use_jsp_value == 'true':
        actions.append(Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time_value}]
        ))

    return actions


def generate_launch_description():
    """Generate the launch description for the robot visualization.

    This function sets up all necessary nodes and parameters for visualizing
    the robot in RViz, including:
    - Robot state publisher for broadcasting transforms
    - Joint state publisher for simulating joint movements
    - RViz for visualization

    Returns:
        LaunchDescription: Complete launch description for the visualization setup
    """
    # Define filenames
    urdf_package = DESCRIPTION_PACKAGE
    urdf_filename = 'rosmaster_x3.urdf.xacro'
    rviz_config_filename = 'autonomous_robot_description.rviz'

    # Set paths to important files
    pkg_share_description = FindPackageShare(urdf_package)
    default_urdf_model_path = PathJoinSubstitution(
        [pkg_share_description, 'urdf', 'robots', urdf_filename])
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share_description, 'rviz', rviz_config_filename])

    # Launch configuration variables
    jsp_gui = LaunchConfiguration('jsp_gui')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_jsp = LaunchConfiguration('use_jsp')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')

    declare_use_jsp_cmd = DeclareLaunchArgument(
        name='use_jsp',
        default_value='false',
        choices=['true', 'false'],
        description='Enable the joint state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    robot_description_content = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'robot_name:=', LaunchConfiguration('robot_name'), ' ',
        'prefix:=', LaunchConfiguration('prefix'), ' ',
        'use_gazebo:=', LaunchConfiguration('use_gazebo')
    ]), value_type=str)

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content}])

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])

    # Create the launch description and populate
    ld = LaunchDescription(ARGUMENTS)

    # Process the controller configuration before starting nodes
    ld.add_action(OpaqueFunction(function=process_ros2_controllers_config))

    # Declare the launch options
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_jsp_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    ld.add_action(OpaqueFunction(function=configure_joint_state_publishers))
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
