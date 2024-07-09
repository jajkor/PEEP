import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(String, 'keystrokes', qos_profile)
        self.timer = self.create_timer(0.5, self.publish_message)

    def publish_message(self):
        while True:
            user_input = input("Enter something: ")
            key = user_input
            msg = String()
            msg.data = key
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % user_input)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()

    try:
        rclpy.spin(keyboard_publisher)
    except KeyboardInterrupt:
        pass

    keyboard_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
