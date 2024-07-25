import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class ServoControl(Node):
    def __init__(self):
        super().__init__('sg90_subscriber')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pwm_channel', rclpy.Parameter.Type.INTEGER),
                ('left_btn', rclpy.Parameter.Type.INTEGER),
                ('right_btn', rclpy.Parameter.Type.INTEGER),
                ('reversed', rclpy.Parameter.Type.BOOL)
            ],
        )
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
    
        self.servo = servo.Servo(self.pca.channels[self.get_parameter('pwm_channel').get_parameter_value().integer_value])
        self.left_btn = self.get_parameter('left_btn').get_parameter_value().integer_value
        self.right_btn = self.get_parameter('right_btn').get_parameter_value().integer_value
        self.reversed = self.get_parameter('reversed').get_parameter_value().bool_value
        self.servo.angle = 90
        
        self.subscription = self.create_subscription(Joy, 'joy', self.cmd_to_angle_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('SG90 Subscriber Initialized')

    def clamp(self, n, min, max):
        if n < min:
            n = min
        elif n > max:
            n = max
        else:
            return n

    def cmd_to_angle_callback(self, msg):
        temp = self.servo.angle

        if (msg.buttons[self.left_btn] == 1) and (msg.buttons[self.right_btn] == 0):
            if self.reversed: 
                temp -= 10
            else: 
                temp += 10

        if (msg.buttons[self.left_btn] == 0) and (msg.buttons[self.right_btn] == 1):
            if self.reversed: 
                temp += 10 
            else: 
                temp -= 10

        self.clamp(temp, 0, 180)
        
        self.servo.angle = temp

        self.get_logger().info(f'Angle: {self.servo.angle}')
        #self.get_logger().info(f'Left: {str(msg.buttons[4])}, Right: {str(msg.buttons[5])}')

        '''
        if (temp > 0):
            if (msg.buttons[self.left_btn] == 1) and (msg.buttons[self.right_btn] == 0):
                if self.reversed: 
                    temp -= 10
                else: 
                    temp += 10
            if (temp < 0):
                temp = 0

        if (temp < 180):
            if (msg.buttons[self.left_btn] == 0) and (msg.buttons[self.right_btn] == 1):
                if self.reversed: 
                    temp += 10 
                else: 
                    temp -= 10
                if (temp > 180):
                    temp = 180
        '''

    #def servo_callback(self, msg):
    #    angle = msg.data
    #    self.servo.angle = angle

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
