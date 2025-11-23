from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Puente para mover el robot (ROS2 -> Gazebo)
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                
                # Puente para la odometría (Gazebo -> ROS2)
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                
                # Puente para las transformaciones TF (Gazebo -> ROS2)
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            output='screen'
        )
    ])