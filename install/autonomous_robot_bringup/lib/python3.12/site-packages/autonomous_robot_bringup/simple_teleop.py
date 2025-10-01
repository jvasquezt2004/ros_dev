#!/usr/bin/env python3
"""Simple velocity publisher for testing robot movement in Docker."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class SimpleVelPublisher(Node):
    def __init__(self):
        super().__init__('simple_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Simple velocity publisher started')
        self.get_logger().info('Use WASD keys to move (Q to quit):')
        self.get_logger().info('W: Forward, S: Backward, A: Left, D: Right')
        self.get_logger().info('Space: Stop, Q: Quit')

    def publish_velocity(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.publisher_.publish(msg)

def get_key():
    """Get a single key press without Enter."""
    try:
        # Store original terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        # Check if key is available
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.read(1)
        else:
            key = ''

        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        return key
    except Exception:
        return ''

def main():
    rclpy.init()
    node = SimpleVelPublisher()

    # Movement speeds
    linear_speed = 0.5

    try:
        while rclpy.ok():
            # Get key input
            key = get_key()

            # Process key
            if key.lower() == 'q':
                break
            elif key.lower() == 'w':
                node.publish_velocity(linear_x=linear_speed)
                node.get_logger().info('Moving forward')
            elif key.lower() == 's':
                node.publish_velocity(linear_x=-linear_speed)
                node.get_logger().info('Moving backward')
            elif key.lower() == 'a':
                node.publish_velocity(linear_y=linear_speed)
                node.get_logger().info('Moving left')
            elif key.lower() == 'd':
                node.publish_velocity(linear_y=-linear_speed)
                node.get_logger().info('Moving right')
            elif key == ' ':
                node.publish_velocity()
                node.get_logger().info('Stopped')

            # Spin once to process callbacks
            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot
        node.publish_velocity()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()