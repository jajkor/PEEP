import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rpi_bot.rpi_interface import RPi_Motors

class Velocity_Subscriber(Node):

    def __init__(self):
        super().__init__('velocity_subscriber')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ena_pin', rclpy.Parameter.Type.INTEGER),
                ('in1_pin', rclpy.Parameter.Type.INTEGER),
                ('in2_pin', rclpy.Parameter.Type.INTEGER),
                ('in3_pin', rclpy.Parameter.Type.INTEGER),
                ('in4_pin', rclpy.Parameter.Type.INTEGER),
                ('enb_pin', rclpy.Parameter.Type.INTEGER),
                ('speed', 50),
                ('differential', 50)
            ],
        )

        self.speed = self.get_parameter('speed').get_parameter_value().integer_value
        self.motors = RPi_Motors(
            self.get_parameter('ena_pin').get_parameter_value().integer_value,
            self.get_parameter('in1_pin').get_parameter_value().integer_value,
            self.get_parameter('in2_pin').get_parameter_value().integer_value,
            self.get_parameter('in3_pin').get_parameter_value().integer_value,
            self.get_parameter('in4_pin').get_parameter_value().integer_value,
            self.get_parameter('enb_pin').get_parameter_value().integer_value
        )
        self.differential = self.get_parameter('differential').get_parameter_value().integer_value

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_to_pwm_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Velocity Subscriber Initialized')

    def cmd_to_pwm_callback(self, msg):
        right_vel = self.speed * msg.linear.x + self.differential * msg.angular.z
        left_vel = self.speed * msg.linear.x - self.differential * msg.angular.z

        self.get_logger().info(f'Received velocities: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
        self.get_logger().info(f'Setting motors: left_vel={left_vel}, right_vel={right_vel}')

        self.motors.setMotors(left_vel, right_vel)

    def destroy_node(self):
        self.get_logger().info('Velocity Subscriber Destroyed')
        self.motors.__del__()

def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = Velocity_Subscriber()
    rclpy.spin(velocity_subscriber)

    velocity_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()