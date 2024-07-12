import rclpy
from geometry_msgs.msg import Twist
from rpi_bot.rpi_interface import RPi_Motors

class Velocity_Subscriber(rclpy.node.Node):

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
                ('speed', 50)
            ],
        )

        self.speed = self.get_parameter('speed')
        self.motors = RPi_Motors(
            self.get_parameter('ena_pin'),
            self.get_parameter('in1_pin'),
            self.get_parameter('in2_pin'),
            self.get_parameter('in3_pin'),
            self.get_parameter('in4_pin'),
            self.get_parameter('enb_pin')
        )

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_to_pwm_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Velocity Subscriber Initialized')

    def cmd_to_pwm_callback(self, msg):
        left_vel = self.speed * (msg.linear.x - msg.angular.z)
        right_vel = self.speed * (msg.linear.x + msg.angular.z)

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