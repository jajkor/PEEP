import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rpi_bot.rpi_interface import RPi_HCS04

class HCS04_Publisher(Node):

    def __init__(self):
        super().__init__('hcs04_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('trig_pin', rclpy.Parameter.Type.INTEGER),
                ('echo_pin', rclpy.Parameter.Type.INTEGER)
            ],
        )

        self.hcs04 = RPi_HCS04(
            self.get_parameter('trig_pin').get_parameter_value().integer_value,
            self.get_parameter('echo_pin').get_parameter_value().integer_value
        )

        self.publisher = self.create_publisher(Range, 'range', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('HC-S04 Publisher Initialized')

    def timer_callback(self):
        distance = self.measure_distance()
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.05  # Assuming a field of view of 5 degrees
        range_msg.min_range = 0.02  # 2 cm
        range_msg.max_range = 4.0  # 4 meters
        range_msg.range = distance

        self.publisher.publish(range_msg)

    def measure_distance(self):
        return self.hcs04.distance

    def destroy_node(self):
        self.get_logger().info('HC-S04 Subscriber Destroyed')
        self.hcs04.__del__()

def main(args=None):
    rclpy.init(args=args)

    hcs04_publisher = HCS04_Publisher()
    rclpy.spin(hcs04_publisher)

    hcs04_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
