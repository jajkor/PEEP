import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rpi_bot.rpi_interface import RPi_SG90

class ServoControl(Node):
    MIN_ANGLE = 0
    MAX_ANGLE = 180
    SPEED = 10

    def __init__(self):
        super().__init__('sg90_driver')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pwm_channel', rclpy.Parameter.Type.INTEGER),
                ('left_btn', rclpy.Parameter.Type.INTEGER),
                ('right_btn', rclpy.Parameter.Type.INTEGER),
                ('reverse', rclpy.Parameter.Type.BOOL),
                ('axes', rclpy.Parameter.Type.BOOL),
                ('axes_btn', rclpy.Parameter.Type.INTEGER)
            ],
        )

        self.left_btn = self.get_parameter('left_btn').get_parameter_value().integer_value
        self.right_btn = self.get_parameter('right_btn').get_parameter_value().integer_value
        self.reverse = self.get_parameter('reverse').get_parameter_value().bool_value
        self.axes = self.get_parameter('axes').get_parameter_value().bool_value
        self.axes_btn = self.get_parameter('axes_btn').get_parameter_value().integer_value
        self.servo.angle = 90

        self.sg90 = RPi_SG90(
            self.get_parameter('pwm_channel').get_parameter_value().integer_value
        )
        
        if self.axes:
            self.subscription = self.create_subscription(Joy, 'joy', self.axes_callback, 10)
        else:
            self.subscription = self.create_subscription(Joy, 'joy', self.btn_callback, 10)

        self.subscription  # prevent unused variable warning
        self.get_logger().info('SG90 Subscriber Initialized')

    def clamp(self, angle, min, max):
        if angle < min:
            return min
        elif angle > max:
            return max
        return angle

    def axes_callback(self, msg):
        temp = self.servo.angle

        if self.axes:
            list = msg.axes[self.axes_btn]
        else:
            list = msg.buttons[self.left_btn, self.right_btn]

        if (msg.axes[self.axes_btn] > 0):
            if self.reverse:
                temp -= ServoControl.SPEED * msg.axes[self.axes_btn]
            else:
                temp += ServoControl.SPEED * msg.axes[self.axes_btn]

        if (msg.axes[self.axes_btn] < 0):
            if self.reverse:
                temp += ServoControl.SPEED * -msg.axes[self.axes_btn]
            else:
                temp -= ServoControl.SPEED * -msg.axes[self.axes_btn]
        
        if temp != self.servo.angle:
            temp = self.clamp(temp, ServoControl.MIN_ANGLE, ServoControl.MAX_ANGLE)
            self.servo.angle = temp
            self.get_logger().info(f'Angle: {self.servo.angle}')

    def btn_callback(self, msg):
        temp = self.servo.angle

        if (msg.buttons[self.left_btn] == 1) and (msg.buttons[self.right_btn] == 0):
            if self.reverse:
                temp -= ServoControl.SPEED
            else:
                temp += ServoControl.SPEED

        if (msg.buttons[self.left_btn] == 0) and (msg.buttons[self.right_btn] == 1):
            if self.reverse:
                temp += ServoControl.SPEED
            else:
                temp -= ServoControl.SPEED

        if temp != self.servo.angle:
            temp = self.clamp(temp, ServoControl.MIN_ANGLE, ServoControl.MAX_ANGLE)
            self.servo.angle = temp
            self.get_logger().info(f'Angle: {self.servo.angle}')

def main(args=None):
    rclpy.init(args=args)

    servo_control = ServoControl()
    rclpy.spin(servo_control)

    servo_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
