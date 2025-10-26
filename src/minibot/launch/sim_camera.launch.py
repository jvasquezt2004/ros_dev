# sim_camera.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    info_topic  = LaunchConfiguration('info_topic')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('info_topic',  default_value='/camera/camera_info'),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                [image_topic, '@sensor_msgs/msg/Image@gz.msgs.Image'],
                [info_topic,  '@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            ],
            output='screen'
        )
    ])
