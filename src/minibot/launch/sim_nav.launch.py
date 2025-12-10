import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap, SetParameter
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Directorios de paquetes
    pkg_minibot = get_package_share_directory('minibot')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # 2. Valores por defecto
    default_map = os.path.join(os.path.expanduser('~'), 'pasillos_mapa.yaml')
    default_params_file = os.path.join(pkg_minibot, 'config', 'nav2_params.yaml')
    nav2_launch_file = os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')

    # 3. LaunchConfigurations (¡instancias!, no tipos)
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )


    # 4. DeclareLaunchArgument
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Ruta completa al archivo YAML del mapa',
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Ruta completa al archivo de parámetros de Nav2',
    )

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar tiempo de simulación (Gazebo) si es true',
    )

    # 5. Nav2 bringup: mapa + localización (AMCL) + navegación
    # En sim_nav.launch.py, dentro de GroupAction
    nav2_stack = TimerAction(
    period=5.0,  # Espera 5 segundos a que AMCL se estabilice
    actions=[
        GroupAction([
            SetRemap('/cmd_vel_nav', '/cmd_vel'),
            SetParameter(name='use_sim_time', value=True),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': 'true',
                    'use_composition': 'True',
                    'slam': 'False',
                    'use_localization': 'True',
                    'map': map_yaml,
                    'use_docking': 'False'
                }.items(),
            ),
        ])
    ]
)

    # 6. Devolver LaunchDescription
    return LaunchDescription(
        [
            declare_map,
            declare_params,
            declare_sim_time,
            static_map_to_odom,
            nav2_stack,
        ]
    )
