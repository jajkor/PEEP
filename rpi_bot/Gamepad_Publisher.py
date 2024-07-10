import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sys, select, termios, tty

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class Gamepad_Publisher(Node):

    def __init__(self):
        super().__init__('gamepad_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(String, 'keystrokes', qos_profile)
        self.timer = self.create_timer(0.5, self.publish_message)


def main(args=None):
    rclpy.init(args=args)
    gamepad_publisher = Gamepad_Publisher()

    try:
        rclpy.spin(gamepad_publisher)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()