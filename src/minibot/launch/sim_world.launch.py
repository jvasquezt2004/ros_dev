import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    pkg_share = get_package_share_directory('minibot')
    
    # 1. Configuración de rutas
    worlds_dir = os.path.join(pkg_share, 'worlds')
    
    # 2. Argumentos
    # Si el usuario no especifica 'world', usamos circuit.sdf
    # Si especifica solo el nombre, PathJoinSubstitution lo unirá con la carpeta worlds
    world_arg = LaunchConfiguration('world')
    
    declare_world = DeclareLaunchArgument(
        'world', 
        default_value=os.path.join(worlds_dir, 'circuit.sdf'),
        description='Ruta completa o nombre del archivo del mundo'
    )
    
    declare_verbosity = DeclareLaunchArgument('verbosity', default_value='4')
    declare_headless = DeclareLaunchArgument('headless', default_value='false')

    # 3. Variable de entorno esencial para que Gazebo encuentre el .dae y el .sdf
    # Añadimos la carpeta 'worlds' a los recursos de Gazebo
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        worlds_dir
    )

    gz_pkg = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')

    # Argumentos para Gazebo
    # Nota: Si pasas una ruta absoluta en 'world', Gazebo la usa.
    # Si pasas un nombre relativo, al haber añadido worlds_dir al RESOURCE_PATH, 
    # Gazebo lo encontrará.
    gz_args = ['-r -v ', LaunchConfiguration('verbosity'), ' ', world_arg]

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': gz_args}.items(),
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': ['-s'] + gz_args}.items(),
        condition=IfCondition(LaunchConfiguration('headless'))
    )

    return LaunchDescription([
        set_gz_resource_path,  # Importante: Esto se ejecuta primero
        declare_world, 
        declare_verbosity, 
        declare_headless,
        gz_gui, 
        gz_server
    ])