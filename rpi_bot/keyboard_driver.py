#!/usr/bin/env python
import sys, select, tty, termios
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    keyboard_publisher = KeyboardPublisher()

    rclpy.spin(keyboard_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_publisher.destroy_node()
    rclpy.shutdown()

    # save old terminal attributes
    old_attr = termios.tcgetattr(sys.stdin)

    # receive input characters as soon as they're pressed
    tty.setcbreak(sys.stdin.fileno())

    print("Publishing keystrokes...")
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key_pub.publish(sys.stdin.read(1))
        rate.sleep()

    # reset terminal attributes when done
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

if __name__ == '__main__':
    main()
