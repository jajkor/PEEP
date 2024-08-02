import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import Range
from rpi_bot.rpi_interface import RPi_SG90
from rpi_bot_interfaces.action import FullScan

class ServoControl(Node):

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

        self.sg90 = RPi_SG90(
            self.get_parameter('pwm_channel').get_parameter_value().integer_value
        )

        self.sg90.set_angle(90)

        self.action_server = ActionServer(self, FullScan, 'full_scan', self.execute_callback)
        self.range_listener = self.create_subscription(Range, 'range', self.range_listener, 10)

        self.range_listener
        self.get_logger().info('SG90 Subscriber Initialized')

    def range_listener(self, range_msg):
        self.distance = range_msg.range * 17150
        self.distance = round(self.distance, 2)

        self.get_logger().info(f'Received Pulse: {range_msg.range}, Calculated Distance: {self.distance} cm')
    
    def get_distance(self):
        return self.distance

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = FullScan.Feedback()
        feedback_msg.current_angles = []
        feedback_msg.current_distances = []

        for i in range (goal_handle.request.start_angle, goal_handle.request.end_angle, 10):
            feedback_msg.current_angles.append(i)
            self.get_logger().info('Feedback: {0}, {0}'.format(feedback_msg.current_angles, feedback_msg.current_distances))
            feedback_msg.current_distances.append(self.get_distance())

            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = FullScan.Result()
        result.list_angles = feedback_msg.current_angles
        result.list_distances = feedback_msg.current_distances
        return result

    def clamp(self, angle, min, max):
        if angle < min:
            return min
        elif angle > max:
            return max
        return angle

def main(args=None):
    rclpy.init(args=args)

    servo_control = ServoControl()
    rclpy.spin(servo_control)

    servo_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
