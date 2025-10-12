# sim_world.launch.py — Gazebo GUI o headless según argumento
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('minibot')
    default_world = os.path.join(pkg_share, 'worlds', 'playground.sdf')

    world = LaunchConfiguration('world')
    verbosity = LaunchConfiguration('verbosity')
    headless = LaunchConfiguration('headless')

    declare_world = DeclareLaunchArgument('world', default_value=default_world)
    declare_verbosity = DeclareLaunchArgument('verbosity', default_value='4')
    declare_headless = DeclareLaunchArgument('headless', default_value='false')

    gz_pkg = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')

    # Con GUI
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': ['-r -v ', verbosity, ' ', world],
        }.items(),
        condition=UnlessCondition(headless)
    )

    # Solo servidor (sin GUI)
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': ['-r -s -v ', verbosity, ' ', world],
        }.items(),
        condition=IfCondition(headless)
    )

    return LaunchDescription([
        declare_world, declare_verbosity, declare_headless,
        gz_gui, gz_server
    ])
