import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rpi_bot.rpi_interface import RPi_HCS04

class HCS04_Publisher(Node):

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

        self.publisher = self.create_publisher(Range, 'range', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info('HC-S04 Driver Initialized')

    def timer_callback(self):
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.radiation_type = Range.ULTRASOUND
        #range_msg.field_of_view = 0.05
        #range_msg.min_range = 0 
        #range_msg.max_range = 1
        range_msg.range = float(self.hcs04.measure_pulse_duration())
        
        self.publisher.publish(range_msg)

    def destroy_node(self):
        self.get_logger().info('HC-S04 Driver Destroyed')
        self.hcs04.__del__()

def main(args=None):
    rclpy.init(args=args)

    hcs04_publisher = HCS04_Publisher()
    rclpy.spin(hcs04_publisher)

    hcs04_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
