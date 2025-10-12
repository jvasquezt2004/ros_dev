# sim_spawn_robot.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
)
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('minibot')

    # ---------- Args ----------
    use_sim_time = LaunchConfiguration('use_sim_time')
    name         = LaunchConfiguration('name')
    x            = LaunchConfiguration('x')
    y            = LaunchConfiguration('y')
    z            = LaunchConfiguration('z')
    yaw          = LaunchConfiguration('yaw')
    xacro_path   = LaunchConfiguration('xacro_path')
    use_r2c      = LaunchConfiguration('use_ros2_control')
    use_r2c_gz   = LaunchConfiguration('use_ros2_control_gz_sim')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_name   = DeclareLaunchArgument('name', default_value='minibot')
    declare_x      = DeclareLaunchArgument('x',   default_value='0.0')
    declare_y      = DeclareLaunchArgument('y',   default_value='0.0')
    declare_z      = DeclareLaunchArgument('z',   default_value='0.1')
    declare_yaw    = DeclareLaunchArgument('yaw', default_value='0.0', description='en radianes')
    declare_xacro  = DeclareLaunchArgument(
        'xacro_path',
        default_value=os.path.join(pkg_share, 'description', 'robot.urdf.xacro'),
        description='Ruta al xacro principal'
    )
    # Por ahora ignoramos ros2_control (lo dejamos parametrizable para el futuro)
    declare_r2c     = DeclareLaunchArgument('use_ros2_control', default_value='false')
    declare_r2c_gz  = DeclareLaunchArgument('use_ros2_control_gz_sim', default_value='false')

    # ---------- robot_description desde xacro ----------
    robot_description = ParameterValue(Command([
        'xacro ', xacro_path,
        ' use_ros2_control:=', use_r2c,
        ' use_ros2_control_gz_sim:=', use_r2c_gz
    ]), value_type=str)

    # ---------- RSP ----------
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # ---------- Spawn en Gazebo ----------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', name,
            '-allow_renaming', 'true',
            '-x', x, '-y', y, '-z', z,
            '-Y', yaw  # radianes
        ]
    )

    # Evita “no such service /world/.../create” si el server tarda en levantar
    delayed_spawn = TimerAction(period=2.0, actions=[spawn])

    # ---------- Recursos para Gazebo ----------
    set_res_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{pkg_share}:" + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    return LaunchDescription([
        declare_use_sim_time, declare_name,
        declare_x, declare_y, declare_z, declare_yaw,
        declare_xacro, declare_r2c, declare_r2c_gz,
        set_res_path,
        rsp,
        delayed_spawn
    ])
