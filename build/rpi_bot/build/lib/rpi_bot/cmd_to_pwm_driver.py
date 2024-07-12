import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from rpi_bot.rpi_interface import RPi_Motors

class Velocity_Subscriber(Node):

    def __init__(self, speed = 100):
        super().__init__('velocity_subscriber')

        self.speed = speed
        self.motors = RPi_Motors()

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