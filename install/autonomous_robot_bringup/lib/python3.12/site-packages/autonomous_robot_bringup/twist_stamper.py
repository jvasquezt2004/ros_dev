"""Convert Twist commands to stamped messages for mecanum controller."""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper(Node):
    """Republish incoming Twist messages as TwistStamped with current time."""

    def __init__(self) -> None:
        super().__init__('twist_stamper')
        qos = rclpy.qos.QoSProfile(depth=10)
        self._pub = self.create_publisher(
            TwistStamped,
            '/mecanum_drive_controller/cmd_vel',
            qos,
        )
        self.create_subscription(Twist, '/cmd_vel', self._callback, qos)

    def _callback(self, msg: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = msg
        self._pub.publish(stamped)


def main() -> None:
    rclpy.init()
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
