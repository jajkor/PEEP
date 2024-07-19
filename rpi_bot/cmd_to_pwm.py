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
                ('differential', 50),
                ('wheel_separation', rclpy.Parameter.Type.DOUBLE),
                ('wheel_radius', rclpy.Parameter.Type.DOUBLE)
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

        self.speed = self.get_parameter('speed').get_parameter_value().integer_value
        self.differential = self.get_parameter('differential').get_parameter_value().integer_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.calculate_wheel_velocity_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Velocity Subscriber Initialized')

    def calculate_wheel_velocity_callback(self, msg):
        #left_vel = self.speed * msg.linear.x - self.differential * msg.angular.z
        left_vel = ((2 * msg.linear.x - msg.angular.z * self.wheel_separation) / (2 * self.wheel_radius))
        #right_vel = self.speed * msg.linear.x + self.differential * msg.angular.z
        right_vel = ((2 * msg.linear.x + msg.angular.z * self.wheel_separation) / (2 * self.wheel_radius))

        pwm_left_vel = self.map_velocity_to_pwm(left_vel, -2.0, 2.0)
        pwm_right_vel = self.map_velocity_to_pwm(right_vel, -2.0, 2.0)

        self.get_logger().info(f'Received velocities: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
        self.get_logger().info(f'Setting motors: left_vel={left_vel}, right_vel={right_vel}')
        self.get_logger().info(f'Setting motors: pwm_left_vel={pwm_left_vel}, pwm_right_vel={pwm_right_vel}')

        self.motors.setMotors(pwm_left_vel, pwm_right_vel)

    def map_velocity_to_pwm(self, velocity, min_velocity, max_velocity):
        """
        Map the velocity to a PWM duty cycle percentage.
        """
        if velocity > max_velocity:
            velocity = max_velocity
        elif velocity < min_velocity:
            velocity = min_velocity
        
        # Map the velocity to a range between 0 and 100 (PWM duty cycle)
        duty_cycle = ((velocity - min_velocity) / (max_velocity - min_velocity)) * 100
        return duty_cycle

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