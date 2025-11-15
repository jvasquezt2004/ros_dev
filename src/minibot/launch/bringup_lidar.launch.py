# bringup_lidar.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # -------- Launch args --------
    serial_port      = LaunchConfiguration('serial_port')
    serial_baudrate  = LaunchConfiguration('serial_baudrate')
    frame_id         = LaunchConfiguration('frame_id')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        # PON AQUÍ tu ruta por defecto actual
        default_value=(
            '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        ),
        description='Puerto serie del RPLIDAR'
    )

    declare_serial_baudrate = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='256000',
        description='Baudrate del RPLIDAR'
    )

    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        # Mejor empatar con el link definido en tu URDF
        default_value='lidar_frame',
        description='Frame TF para los datos del LIDAR'
    )

    # -------- Incluir el launch de rplidar_ros --------
    rplidar_share = get_package_share_directory('rplidar_ros')
    rplidar_launch_path = os.path.join(
        rplidar_share, 'launch', 'rplidar.launch.py'
    )

    rplidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_path),
        launch_arguments={
            'serial_port':      serial_port,
            'serial_baudrate':  serial_baudrate,
            'frame_id':         frame_id,
        }.items(),
    )

    return LaunchDescription([
        declare_serial_port,
        declare_serial_baudrate,
        declare_frame_id,
        rplidar_include,
    ])
