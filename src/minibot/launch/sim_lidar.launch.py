# sim_lidar.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    laser_topic  = LaunchConfiguration('laser_topic')
    points_topic = LaunchConfiguration('points_topic')

    return LaunchDescription([
        DeclareLaunchArgument('laser_topic',  default_value='/scan'),
        DeclareLaunchArgument('points_topic', default_value='/scan/points'),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                [laser_topic,  '@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
                [points_topic, '@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
            ],
            output='screen'
        )
    ])
