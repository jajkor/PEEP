import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rpi_bot.rpi_interface import RPi_SG90
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from rpi_bot_interfaces.action import Scan
import time

class ServoControl(Node):

    def __init__(self):
        super().__init__('sg90_driver')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pwm_channel', rclpy.Parameter.Type.INTEGER),
                ('reversed', rclpy.Parameter.Type.BOOL)
            ],
        )

        self.sg90 = RPi_SG90(
            self.get_parameter('pwm_channel').get_parameter_value().integer_value,
            90
        )

        self.is_running = True
        self.reversed = self.get_parameter('reversed').get_parameter_value().bool_value

        self.action_server = ActionServer(self, Scan, 'scan', self.execute_callback)

        #self.start_service = self.create_service(Empty, 'start', self.start_callback)
        #self.stop_service = self.create_service(Empty, 'stop', self.stop_callback)

        self.get_logger().info('SG90 Subscriber Initialized')

    def execute_callback(self, goal_handle):
        if self.is_running:
            self.get_logger().info('Executing goal...')
            feedback_msg = Scan.Feedback()
            result_msg = Scan.Result()

            for i in range(int(goal_handle.request.start_angle), int(goal_handle.request.stop_angle), 10):
                time.sleep(0.5)
                self.sg90.set_angle(i)

                feedback_msg.current_angle = float(self.sg90.get_angle())
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f'Feedback: {feedback_msg.current_angle}')
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    return Scan.Result()

            result_msg.final_angle = feedback_msg.current_angle
            goal_handle.succeed()
            self.get_logger().info('Goal succeeded')
            return result_msg
        else:
            self.get_logger().info('Not Running')

    def start_callback(self, request, response):
        if not self.is_running:
            self.is_running = True
            self.get_logger().info('Node started')
        else:
            self.get_logger().info('Node is already running')
        return response

    def stop_callback(self, request, response):
        if self.is_running:
            self.is_running = False
            self.get_logger().info('Node stopped')
        else:
            self.get_logger().info('Node is already stopped')
        return response

def main(args=None):
    rclpy.init(args=args)

    servo_control = ServoControl()
    rclpy.spin(servo_control)

    servo_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
