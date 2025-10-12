# sim_world_robot.launch.py — one-stop bringup (world + robot + lidar + rviz)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def pkg_share(pkg: str) -> str:
    return get_package_share_directory(pkg)


def generate_launch_description():
    # ----------------
    # Launch arguments
    # ----------------
    world_default = os.path.join(pkg_share('minibot'), 'worlds', 'playground.sdf')
    rviz_default  = os.path.join(pkg_share('minibot'), 'rviz', 'sim_lidar.rviz')
    xacro_default = os.path.join(pkg_share('minibot'), 'description', 'robot.urdf.xacro')

    declare_headless   = DeclareLaunchArgument('headless',     default_value='false')
    declare_verbosity  = DeclareLaunchArgument('verbosity',    default_value='4')
    declare_world      = DeclareLaunchArgument('world',        default_value=world_default)
    declare_name       = DeclareLaunchArgument('name',         default_value='minibot')
    declare_x          = DeclareLaunchArgument('x',            default_value='0.0')
    declare_y          = DeclareLaunchArgument('y',            default_value='0.0')
    declare_z          = DeclareLaunchArgument('z',            default_value='0.1')
    declare_yaw        = DeclareLaunchArgument('yaw',          default_value='0.0')
    declare_xacro      = DeclareLaunchArgument('xacro',        default_value=xacro_default)

    declare_use_lidar  = DeclareLaunchArgument('use_lidar',    default_value='true')
    declare_use_rviz   = DeclareLaunchArgument('use_rviz',     default_value='true')
    declare_rviz_cfg   = DeclareLaunchArgument('rviz_config',  default_value=rviz_default)

    headless   = LaunchConfiguration('headless')
    verbosity  = LaunchConfiguration('verbosity')
    world      = LaunchConfiguration('world')
    name       = LaunchConfiguration('name')
    x          = LaunchConfiguration('x')
    y          = LaunchConfiguration('y')
    z          = LaunchConfiguration('z')
    yaw        = LaunchConfiguration('yaw')
    xacro      = LaunchConfiguration('xacro')
    use_lidar  = LaunchConfiguration('use_lidar')
    use_rviz   = LaunchConfiguration('use_rviz')
    rviz_cfg   = LaunchConfiguration('rviz_config')

    # ----------------
    # Environment fixes (Wayland/Hyprland + OGRE)
    # ----------------
    # Forzar backend XCB (XWayland) para Qt, limpiar estilos y overrides de Mesa
    force_xcb       = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    clear_qt_style  = SetEnvironmentVariable('QT_STYLE_OVERRIDE', '')
    clear_mesa_gl   = SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '')
    clear_mesa_glsl = SetEnvironmentVariable('MESA_GLSL_VERSION_OVERRIDE', '')
    # Workaround de RViz/OGRE cuando mueves la ventana entre escritorios
    ogre_rtt_copy   = SetEnvironmentVariable('OGRE_RTT_MODE', 'Copy')

    # ----------------
    # Include: world (gz sim)
    # ----------------
    include_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share('minibot'), 'launch', 'sim_world.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbosity': verbosity,
            'headless': headless,   # ahora sí respeta headless (ver archivo 2)
        }.items(),
    )

    # ----------------
    # Include: spawn robot (también publica robot_description)
    # ----------------
    include_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share('minibot'), 'launch', 'sim_spawn_robot.launch.py')
        ),
        launch_arguments={
            'name': name,
            'x': x, 'y': y, 'z': z, 'yaw': yaw,
            'xacro': xacro,
        }.items(),
    )

    # ----------------
    # Include: lidar bridge (ROS <-> GZ)
    # ----------------
    include_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share('minibot'), 'launch', 'sim_lidar.launch.py')
        ),
        condition=IfCondition(use_lidar),
    )

    # ----------------
    # RViz2 (opcional)
    # ----------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(use_rviz),
        # Reaplicamos los env críticos directamente al proceso de RViz
        additional_env={
            'QT_QPA_PLATFORM': 'xcb',
            'QT_STYLE_OVERRIDE': '',
            'MESA_GL_VERSION_OVERRIDE': '',
            'MESA_GLSL_VERSION_OVERRIDE': '',
            'OGRE_RTT_MODE': 'Copy',
            # Si te interesa forzar software GL solo para RViz:
            # 'LIBGL_ALWAYS_SOFTWARE': '1',
            # En algunas GPUs Wayland ayuda también:
            # 'LIBGL_DRI3_DISABLE': '1',
        },
    )

    return LaunchDescription([
        # Args
        declare_world, declare_verbosity, declare_headless,
        declare_name, declare_x, declare_y, declare_z, declare_yaw,
        declare_xacro, declare_use_lidar, declare_use_rviz, declare_rviz_cfg,
        # Env saneado
        force_xcb, clear_qt_style, clear_mesa_gl, clear_mesa_glsl, ogre_rtt_copy,
        # Bringup
        include_world,
        include_spawn,
        include_lidar,
        rviz,
    ])
