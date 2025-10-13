from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            output='screen',
            arguments=[
                '/world/playground/model/minibot/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/world/playground/model/minibot/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
        ),
    ])