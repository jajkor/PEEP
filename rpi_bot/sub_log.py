import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HCS04_Subscriber(Node):

    def __init__(self):
        super().__init__('log_subscriber')
        
        self.subscription = self.create_subscription(String, 'sens_dist', self.hcs04_callback, 10)
        self.subscription = self.create_subscription(String, 'joy', self.sg90_callback, 10)
        self.subscription = self.create_subscription(String, 'cmd_vel', self.controller_callback, 10)
        
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Log Subscriber Initialized')

    def hcs04_callback(self, msg):
        self.get_logger().info(f'HC-S04 Distance: {msg.data} cm')

    def sg90_callback(self, msg):
        self.get_logger().info(f'SG90 Angle: {msg.data}')

    def controller_callback(self, msg):
        self.get_logger().info(f'Received velocities: linear.x={msg.linear.x}, angular.z={msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)

    hcs04_subscriber = HCS04_Subscriber()
    rclpy.spin(hcs04_subscriber)

    hcs04_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
