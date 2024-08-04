import time
import rclpy
from rclpy.node import Node
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
                ('starting_angle', 90)
            ],
        )

        self.sg90 = RPi_SG90(
            self.get_parameter('pwm_channel').get_parameter_value().integer_value,
            self.get_parameter('starting_angle').get_parameter_value().integer_value
        )
        self.is_busy = False
        self.srv = self.create_service(Scan, 'servo_scan', self.scan_callback)
        self.range_listener = self.create_subscription(Range, 'range', self.range_listener, 10)

    def range_listener(self, range_msg):
        self.distance = range_msg.range * 17150
        self.distance = round(self.distance, 2)

    def scan_callback(self, request, response):
        if self.is_busy:
            self.get_logger().info('Service is busy. Cannot handle request: %s' % request.request_data)
            response.response_data = 'Service is currently busy. Please try again later.'
            return response
        
        self.is_busy = True
        self.get_logger.info(f"Servo Scan: {request.start_angle} to {request.stop_angle}")

        for i in range(int(request.start_angle), int(request.stop_angle), 10):
            self.sg90.set_angle(i)
            response.list_angles.append(float(self.sg90.get_angle()))
            response.list_distances.append(float(self.distance))
            time.sleep(0.5)

        self.is_busy = False
        return response

def main(args=None):
    rclpy.init(args=args)

    servo_scan = Servo_Scan()
    rclpy.spin(servo_scan)

    servo_scan.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
