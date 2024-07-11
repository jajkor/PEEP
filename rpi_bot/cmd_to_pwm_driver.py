import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rpi_bot.motor_interface import RPi_Motors
import RPi.GPIO as GPIO

class Velocity_Subscriber(Node):

    def __init__(self):
        super().__init__('velocity_subscriber')
        motors = RPi_Motors()
        motors.setPWMA(100)
        motors.setPWMB(100)

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_to_pwm_callback, 10)
        self.subscription  # prevent unused variable warning

    def cmd_to_pwm_callback(self, msg):
        right_wheel_vel = (msg.linear.x + msg.angular.z) / 2
        left_wheel_vel = (msg.linear.x - msg.angular.z) / 2

        # Right Motor
        GPIO.output(self.IN1, right_wheel_vel > 0)
        GPIO.output(self.in2, right_wheel_vel < 0)

        # Left Motor
        GPIO.output(self.IN3, left_wheel_vel > 0)
        GPIO.output(self.IN4, left_wheel_vel < 0)
        print(right_wheel_vel, ' / ', left_wheel_vel)
        #self.get_logger().info('I heard: "%s" and "%s"' % right_wheel_vel, left_wheels_vel)


def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = Velocity_Subscriber()

    rclpy.spin(velocity_subscriber)

    velocity_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()