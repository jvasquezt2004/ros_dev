import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_minibot = get_package_share_directory('minibot')
    
    # 1. Argumento para usar Sim Time (CRÍTICO para arreglar tu error)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 2. Archivo de configuración de RViz para SLAM
    # Usaremos uno nuevo o el que ya tienes, pero nos aseguramos de cargarlo
    rviz_config_dir = os.path.join(pkg_minibot, 'rviz', 'sim_slam.rviz')

    return LaunchDescription([
        # Nodo RViz2 con parámetro use_sim_time
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_slam',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}], # <--- ESTO ARREGLA EL QUEUE FULL
            output='screen'
        ),
    ])