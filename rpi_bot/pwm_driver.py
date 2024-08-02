import rclpy
from rclpy.node import Node
from rpi_bot.rpi_interface import RPi_Motors
from rpi_bot_interfaces.msg import Velocity

class PWM_Driver(Node):

    def __init__(self):
        super().__init__('pwm_driver')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ena_pin', rclpy.Parameter.Type.INTEGER),
                ('in1_pin', rclpy.Parameter.Type.INTEGER),
                ('in2_pin', rclpy.Parameter.Type.INTEGER),
                ('in3_pin', rclpy.Parameter.Type.INTEGER),
                ('in4_pin', rclpy.Parameter.Type.INTEGER),
                ('enb_pin', rclpy.Parameter.Type.INTEGER),
            ],
        )

        self.motors = RPi_Motors(
            self.get_parameter('ena_pin').get_parameter_value().integer_value,
            self.get_parameter('in1_pin').get_parameter_value().integer_value,
            self.get_parameter('in2_pin').get_parameter_value().integer_value,
            self.get_parameter('in3_pin').get_parameter_value().integer_value,
            self.get_parameter('in4_pin').get_parameter_value().integer_value,
            self.get_parameter('enb_pin').get_parameter_value().integer_value
        )

        self.left = 0
        self.right = 0

        self.subscription = self.create_subscription(Velocity, 'motor_vel', self.velocity_listener, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('PWM Driver Initialized')

    def velocity_listener(self, vel_msg):
        if (self.left != vel_msg.left_vel) and (self.left != vel_msg.left_vel):
            self.motors.setMotors(vel_msg.left_vel, vel_msg.right_vel)
            self.get_logger().info(f'Setting Velocity: left={vel_msg.left_vel}, right={vel_msg.right_vel}')

        self.left = vel_msg.left_vel
        self.right = vel_msg.right_vel

def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = PWM_Driver()
    rclpy.spin(velocity_subscriber)

    velocity_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()