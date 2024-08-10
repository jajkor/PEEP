import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Range
from rpi_bot.rpi_interface import RPi_HCS04

class HCS04_Driver(Node):

    def __init__(self):
        super().__init__('hcs04_driver')

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

        self.range_publisher = self.create_publisher(Range, 'range', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info('HC-S04 Driver Initialized')

    def calculate_distance_cm(self, pulse) -> float:
        return round(float(pulse * 17150), 2)

    def timer_callback(self):
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.range = self.calculate_distance_cm(self.hcs04.measure_pulse_duration())

        self.range_publisher.publish(range_msg)
        self.get_logger().info(f'Publishing Distance: {range_msg.range} cm')

def main(args=None):
    rclpy.init(args=args)

    hcs04_publisher = HCS04_Driver()
    try:
        rclpy.spin(hcs04_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        hcs04_publisher.destroy_node()


if __name__ == '__main__':
    main()
