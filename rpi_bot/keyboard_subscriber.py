#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#speed = 50
#differential = 50
#bot = RPi_Bot()

class KeyboardSubscriber(Node):

    def __init__(self):
        super().__init__('keyboard_subscriber')
        self.subscription = self.create_subscription(String, 'keystrokes', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

#def command_callback(twist):
    #left_speed = speed * twist.linear.x - differential * twist.angular.z
    #right_speed = speed * twist.linear.x + differential * twist.angular.z
    #bot.setMotor(left_speed, right_speed)

def main(args=None):
    rclpy.init(args=args)

    keyboard_subscriber = KeyboardSubscriber()

    rclpy.spin(keyboard_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
