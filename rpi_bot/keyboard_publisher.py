import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sys, select, termios, tty
settings = termios.tcgetattr(sys.stdin)

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class Keyboard_Publisher(Node):

    def __init__(self):
        super().__init__('keyboard_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(String, 'keystrokes', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_message)

    def publish_message(self):
        user_input = read_key()
        key = user_input
        msg = String()
        msg.data = key
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % user_input)


def read_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = Keyboard_Publisher()

    try:
        rclpy.spin(keyboard_publisher)
    except KeyboardInterrupt:
        pass

    keyboard_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
