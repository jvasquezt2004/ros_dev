#! /usr/bin/env python3

"""
Publish a string "Hello, world!" to a topic

-----
Publishing Topics:
  The channel containing the "Hello World" messages
  /py_example_topic - std_msgs/String

Subscription Topics:
  None
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPyPublisher(Node):
  def __init__(self):
    super().__init__('minimal_py_publisher')

    self.publisher_1 = self.create_publisher(String, '/py_example_topic', 10)
    self.timer = self.create_timer(0.5, self.timer_callback)

    self.i = 0


  def timer_callback(self):
    msg = String()

    msg.data = 'Hello, world: %d' % self.i

    self.publisher_1.publish(msg)

    self.get_logger().info('Publishing: "%s"' % msg.data)

    self.i += 1


def main(args=None):
  rclpy.init(args=args)
  minimal_py_publisher = MinimalPyPublisher()
  rclpy.spin(minimal_py_publisher)
  minimal_py_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
