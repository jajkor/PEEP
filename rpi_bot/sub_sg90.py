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

        self.angle = 0

        self.subscription = self.create_subscription(Joy, 'joy', self.cmd_to_angle_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('SG90 Subscriber Initialized')

    def cmd_to_angle_callback(self, msg):
        if (self.angle > -90):
            if (msg.buttons[4] == 1) and (msg.buttons[5] == 0):
                self.angle -= 1
        if (self.angle < 90):
            if (msg.buttons[4] == 0) and (msg.buttons[5] == 1):
                self.angle += 1

        self.sg90.set_angle(self.angle)

        self.get_logger().info(f'Angle: {self.angle}')
        self.get_logger().info(f'Left: {str(msg.buttons[4])}, Right: {str(msg.buttons[5])}')


def main(args=None):
    rclpy.init(args=args)

    hcs04_subscriber = SG90_Subscriber()
    rclpy.spin(hcs04_subscriber)

    hcs04_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
