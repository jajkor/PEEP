import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class ServoControl(Node):
    def __init__(self):
        super().__init__('servo_control')
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
        self.servo = servo.Servo(self.pca.channels[0])
        self.subscription = self.create_subscription(
            Float32,
            'servo_angle',
            self.servo_callback,
            10)
        self.subscription  # prevent unused variable warning

    def servo_callback(self, msg):
        angle = msg.data
        self.servo.angle = angle

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
