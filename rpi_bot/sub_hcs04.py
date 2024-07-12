import rclpy
from std_msgs.msg import String

class HCS04_Subscriber(rclpy.Node):

    def __init__(self):
        super().__init__('hcs04_subscriber')
        
        self.subscription = self.create_subscription(String, 'sens_dist', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('HC-S04 Subscriber Initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Distance: {msg.data} cm')


def main(args=None):
    rclpy.init(args=args)

    hcs04_subscriber = HCS04_Subscriber()
    rclpy.spin(hcs04_subscriber)

    hcs04_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
