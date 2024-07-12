import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rpi_bot.rpi_interface import RPi_HCS04

class HCS04_Publisher(Node):

    def __init__(self):
        super().__init__('hcs04_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('trig_pin', rclpy.Parameter.Type.INTEGER),
                ('echo_pin', rclpy.Parameter.Type.INTEGER),
            ],
        )

        self.hcs04 = RPi_HCS04(
            self.get_parameter('trig_pin'),
            self.get_parameter('echo_pin')
        )

        self.i = 0

        self.publisher = self.create_publisher(String, 'sens_dist', 10)
        timer_period = 0.038  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('HC-S04 Publisher Initialized')

    def timer_callback(self):
        msg = String()
        msg.data = '%d' %self.hcs04.distance()
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing Distance: {msg.data} cm')
        self.i += 1

    def destroy_node(self):
        self.get_logger().info('HC-S04 Subscriber Destroyed')
        self.hcs04.__del__()

# May need to change 'args=None'
def main(args=None):
    rclpy.init(args=args)

    hcs04_publisher = HCS04_Publisher()
    rclpy.spin(hcs04_publisher)

    hcs04_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
