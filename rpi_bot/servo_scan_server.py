import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rpi_bot.rpi_interface import RPi_SG90
from rpi_bot_interfaces.srv import Scan
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class Servo_Scan(Node):

    def __init__(self) -> None:
        super().__init__('servo_scan')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pwm_channel', rclpy.Parameter.Type.INTEGER),
                ('starting_angle', 90)
            ],
        )

        self.sg90 = RPi_SG90(
            self.get_parameter('pwm_channel').get_parameter_value().integer_value,
            self.get_parameter('starting_angle').get_parameter_value().integer_value
        )
        self.is_busy = False
        self.distance = None

        self.callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(Scan, 'servo_scan', self.scan_callback, callback_group=self.callback_group)
        self.range_listener = self.create_subscription(Range, 'range', self.range_listener_callback, 10,  callback_group=self.callback_group)

    def range_listener_callback(self, range_msg):
        self.distance = range_msg.range * 17150
        self.distance = round(self.distance, 2)
        self.get_logger().info(f'Updated distance: {self.distance}')

    def scan_callback(self, request, response):
        if self.is_busy:
            self.get_logger().info('Service is busy. Cannot handle request:')
            response.response_data = 'Service is currently busy. Please try again later.'
            return response
        
        self.is_busy = True
        self.get_logger().info(f"Servo Scan: {request.start_angle} to {request.stop_angle}")

        for i in range(int(request.start_angle), int(request.stop_angle), 10):
            self.sg90.set_angle(i)
            time.sleep(0.5)
            response.list_angle.append(float(self.sg90.get_angle()))
            response.list_distance.append(float(self.distance))

        self.is_busy = False
        self.sg90.set_angle(90)

        self.get_logger().info(f'List_Angle: {response.list_angle}')
        self.get_logger().info(f'List_Distance: {response.list_distance}')
        return response

def main(args=None):
    rclpy.init(args=args)

    servo_scan = Servo_Scan()
    executor = MultiThreadedExecutor()
    executor.add_node(servo_scan)

    try:
        servo_scan.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        servo_scan.get_logger().info('Keyboard interrupt, shutting down.\n')

    servo_scan.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
