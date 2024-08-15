import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Range
from rpi_bot.rpi_interface import RPi_SG90
from rpi_bot_interfaces.srv import Scan

class Servo_Scan(Node):

    def __init__(self) -> None:
        super().__init__('servo_scan')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pwm_channel', rclpy.Parameter.Type.INTEGER),
                ('start_angle', 90),
                ('scan_speed', 0.2),
            ],
        )

        self.sg90 = RPi_SG90(
            self.get_parameter('pwm_channel').get_parameter_value().integer_value,
            self.get_parameter('start_angle').get_parameter_value().integer_value
        )

        self.start_angle = self.get_parameter('start_angle').get_parameter_value().integer_value
        self.sg90.set_angle(self.start_angle)
        self.scan_speed = self.get_parameter('scan_speed').get_parameter_value().double_value
        self.get_logger().info(f'{self.scan_speed}')

        self.distance = None

        self.servo_scan_callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(Scan, 'servo_scan', self.scan_callback, callback_group=self.servo_scan_callback_group)
        self.range_listener = self.create_subscription(Range, 'range', self.range_listener_callback, 10, callback_group=self.servo_scan_callback_group)
        self.get_logger().info('Servo Scan Server Initialized')


    def range_listener_callback(self, range_msg):
        self.distance = range_msg.range

    def scan_callback(self, request, response):
        self.get_logger().info(f"Servo Scan: {request.start_angle} to {request.stop_angle}")

        for i in range(int(request.start_angle), int(request.stop_angle), 10):
            self.sg90.set_angle(i)
            time.sleep(self.scan_speed)
            response.list_angle.append(round(float(self.sg90.get_angle()), 2))
            response.list_distance.append(self.distance)

        self.sg90.set_angle(self.start_angle)

        self.get_logger().info(f'Angle: {response.list_angle}')
        self.get_logger().info(f'Distance: {response.list_distance}')
        
        return response

def main(args=None):
    rclpy.init(args=args)

    servo_scan = Servo_Scan()
    servo_scan_executor = MultiThreadedExecutor()
    servo_scan_executor.add_node(servo_scan)

    try:
        servo_scan_executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        servo_scan.destroy_node()
        
if __name__ == '__main__':
    main()
