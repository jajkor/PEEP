import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup

from rpi_bot.rpi_interface import RPi_SG90
from rpi_bot_interfaces.srv import Scan

class Servo_Scan(Node):

    def __init__(self) -> None:
        super().__init__('servo_scan')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pwm_channel', rclpy.Parameter.Type.INTEGER),
                ('start_angle', 90)
            ],
        )

        self.sg90 = RPi_SG90(
            self.get_parameter('pwm_channel').get_parameter_value().integer_value,
            self.get_parameter('start_angle').get_parameter_value().integer_value
        )

        self.start_angle = self.get_parameter('start_angle').get_parameter_value().integer_value
        self.sg90.set_angle(self.start_angle)

        self.is_busy = False

        self.callback_group = ReentrantCallbackGroup()
        self.scan_srv = self.create_service(Scan, 'servo_scan', self.scan_callback, callback_group=self.callback_group)

    def scan_callback(self, request, response):
        if self.is_busy:
                self.get_logger().warn('Servo is busy. Ignoring new request.')
                return response  # Optionally set an error status in the response

        self.is_busy = True

        self.get_logger().info(f"Servo Scan: {request.start_angle} to {request.stop_angle}")

        for i in range(int(request.start_angle), int(request.stop_angle), 10):
            self.sg90.set_angle(i)
            time.sleep(0.5)
            response.list_angle.append(round(float(self.sg90.get_angle()), 2))

        self.sg90.set_angle(self.start_angle)
        self.is_busy = False

        self.get_logger().info(f'List_Angle: {response.list_angle}')
        
        return response

def main(args=None):
    rclpy.init(args=args)

    servo_scan = Servo_Scan()

    try:
        rclpy.spin(servo_scan)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        servo_scan.destroy_node()

if __name__ == '__main__':
    main()
