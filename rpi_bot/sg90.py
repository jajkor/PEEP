import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class ServoControl(Node):
    MIN_ANGLE = 0
    MAX_ANGLE = 180
    SPEED = 10

    def __init__(self):
        super().__init__('sg90_subscriber')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pwm_channel', rclpy.Parameter.Type.INTEGER),
                ('left_btn', rclpy.Parameter.Type.INTEGER),
                ('right_btn', rclpy.Parameter.Type.INTEGER),
                ('reverse', rclpy.Parameter.Type.BOOL),
                ('axes', rclpy.Parameter.Type.BOOL)
            ],
        )
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address = 0x40)
        self.pca.frequency = 50
    
        self.servo = servo.Servo(self.pca.channels[self.get_parameter('pwm_channel').get_parameter_value().integer_value])
        self.left_btn = self.get_parameter('left_btn').get_parameter_value().integer_value
        self.right_btn = self.get_parameter('right_btn').get_parameter_value().integer_value
        self.reverse = self.get_parameter('reverse').get_parameter_value().bool_value
        self.axes = self.get_parameter('axes').get_parameter_value().bool_value
        self.servo.angle = 90
        
        if self.axes:
            self.subscription = self.create_subscription(Joy, 'joy', self.axes_callback, 10)
        else:
            self.subscription = self.create_subscription(Joy, 'joy', self.btn_callback, 10)

        self.subscription  # prevent unused variable warning
        self.get_logger().info('SG90 Subscriber Initialized')

    def clamp(self, n, min, max):
        if n < min:
            return min
        elif n > max:
            return max
        return n

    def axes_callback(self, msg):
        temp = self.servo.angle

        if (msg.axes[self.left_btn] >= 0):
            if self.reverse:
                temp -= ServoControl.SPEED
            else:
                temp += ServoControl.SPEED

        if (msg.axes[self.left_btn] >= -0):
            if self.reverse:
                temp += ServoControl.SPEED
            else:
                temp -= ServoControl.SPEED

        temp = self.clamp(temp, ServoControl.MIN_ANGLE, ServoControl.MAX_ANGLE)
        self.servo.angle = temp
        self.get_logger().info(f'Angle: {self.servo.angle}')
        self.get_logger().info(f'Axes: {msg.axes[self.left_btn]}')

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

        temp = self.clamp(temp, ServoControl.MIN_ANGLE, ServoControl.MAX_ANGLE)
        self.servo.angle = temp
        self.get_logger().info(f'Angle: {self.servo.angle}')

    def destroy(self):
        self.pca.deinit()

def main(args=None):
    rclpy.init(args=args)

    servo_control = ServoControl()
    rclpy.spin(servo_control)

    servo_control.destroy()
    servo_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
