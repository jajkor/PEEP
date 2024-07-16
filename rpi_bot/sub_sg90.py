import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rpi_bot.rpi_interface import RPi_SG90

class SG90_Subscriber(Node):

    def __init__(self):
        super().__init__('sg90_subscriber')
        
        self.declare_parameter('pwm_pin', rclpy.Parameter.Type.INTEGER)

        self.sg90 = RPi_SG90(
            self.get_parameter('pwm_pin').get_parameter_value().integer_value
        )

        self.subscription = self.create_subscription(Joy, 'joy', self.cmd_to_pwm_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('SG90 Subscriber Initialized')

    def cmd_to_pwm_callback(self, msg):
        self.get_logger().info('Buttons: "%s"' % str(msg.buttons))


def main(args=None):
    rclpy.init(args=args)

    hcs04_subscriber = SG90_Subscriber()
    rclpy.spin(hcs04_subscriber)

    hcs04_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
