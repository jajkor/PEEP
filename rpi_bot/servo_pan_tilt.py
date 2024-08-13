import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Joy
from rpi_bot.rpi_interface import RPi_SG90

class ServoControl(Node):
    MIN_ANGLE = 0
    MAX_ANGLE = 180
    SPEED = float(10.0)

    def __init__(self):
        super().__init__('servo_pan_tilt')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pan_pwm_channel', rclpy.Parameter.Type.INTEGER),
                ('pan_start_angle', 90),
                ('pan_axes', rclpy.Parameter.Type.INTEGER),
                ('tilt_pwm_channel', rclpy.Parameter.Type.INTEGER),
                ('tilt_start_angle', 90),
                ('tilt_axes', rclpy.Parameter.Type.INTEGER),
            ],
        )

        self.pan_servo = RPi_SG90(
            self.get_parameter('pan_pwm_channel').get_parameter_value().integer_value,
            self.get_parameter('pan_start_angle').get_parameter_value().double_value,
        )
        self.tilt_servo = RPi_SG90(
            self.get_parameter('tilt_pwm_channel').get_parameter_value().integer_value,
            self.get_parameter('tilt_start_angle').get_parameter_value().double_value,
        )

        self.pan_start_angle = self.get_parameter('pan_start_angle').get_parameter_value().double_value
        self.tilt_start_angle = self.get_parameter('tilt_start_angle').get_parameter_value().double_value

        self.pan_servo.set_angle(self.pan_start_angle)
        self.tilt_servo.set_angle(self.tilt_start_angle)

        self.pan_axes = self.get_parameter('pan_axes').get_parameter_value().integer_value
        self.tilt_axes = self.get_parameter('tilt_axes').get_parameter_value().integer_value

        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        self.get_logger().info('SG90 Subscriber Initialized')

    def clamp(self, n, min, max):
        if n < min:
            return min
        elif n > max:
            return max
        return n

    def joy_callback(self, msg):
        pan_temp = float(self.pan_servo.get_angle())
        tilt_temp = float(self.tilt_servo.get_angle())

        if (msg.axes[self.pan_axes] > 0):
            pan_temp -= ServoControl.SPEED * -float(msg.axes[self.pan_axes])
        else:
            pan_temp += ServoControl.SPEED * float(msg.axes[self.pan_axes])

        if (msg.axes[self.tilt_axes] > 0):
            tilt_temp += ServoControl.SPEED * float(msg.axes[self.tilt_axes])
        else:
            tilt_temp -= ServoControl.SPEED * -float(msg.axes[self.tilt_axes])
        
        if pan_temp != self.pan_servo.get_angle():
            pan_temp = self.clamp(pan_temp, ServoControl.MIN_ANGLE, ServoControl.MAX_ANGLE)
            self.pan_servo.set_angle(pan_temp)
            self.get_logger().info(f'Pan Angle: {self.pan_servo.get_angle()}')

        if tilt_temp != self.tilt_servo.get_angle():
            tilt_temp = self.clamp(tilt_temp, ServoControl.MIN_ANGLE, ServoControl.MAX_ANGLE)
            self.tilt_servo.set_angle(tilt_temp)
            self.get_logger().info(f'Tilt Angle: {self.tilt_servo.get_angle()}')

def main(args=None):
    rclpy.init(args=args)

    servo_control = ServoControl()
    try:
        rclpy.spin(servo_control)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        servo_control.destroy_node()

if __name__ == '__main__':
    main()