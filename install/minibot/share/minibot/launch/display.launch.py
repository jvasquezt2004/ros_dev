# ~/ros2_ws/src/minibot/launch/display.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # ---- 1. Localizar los archivos necesarios ----
    # Obtener la ruta de la carpeta del paquete
    pkg_path = get_package_share_directory('minibot')

    # Obtener la ruta del archivo de configuración de RViz
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'display.rviz')

    # Obtener la ruta del archivo XACRO principal de tu robot
    xacro_file_path = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # ---- 2. Declarar los argumentos de lanzamiento ----
    # Este argumento permite usar o no el GUI para controlar las articulaciones
    use_joint_state_publisher_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui')

    # ---- 3. Definir los Nodos que se van a lanzar ----

    # Nodo: robot_state_publisher
    # Procesa el archivo XACRO para generar el URDF, lo carga como un parámetro
    # y publica las transformaciones (TF2) del robot.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file_path]),
                value_type=str
            ),
        }]
    )

    # Nodo: joint_state_publisher_gui
    # Proporciona una interfaz gráfica con sliders para mover las articulaciones del robot.
    # Es muy útil para probar el modelo.
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Nodo: rviz2
    # Lanza la herramienta de visualización RViz2 con una configuración predefinida.
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # ---- 4. Crear la descripción del lanzamiento ----
    # Esta es la lista de todas las acciones y nodos que se ejecutarán.
    return LaunchDescription([
        use_joint_state_publisher_gui,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])