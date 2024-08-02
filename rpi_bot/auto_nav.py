import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Range
from rpi_bot_interfaces.msg import Velocity
from rpi_bot_interfaces.action import FullScan

class Auto_Nav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        self.speed = 75
        self.differential = 75
        self.linear = 0
        self.angular = 0
        self.distance = 0

        #self.range_listener = self.create_subscription(Range, 'range', self.range_listener, 10)
        self.velocity_publisher = self.create_publisher(Velocity, 'motor_vel', 10)
        self.timer = self.create_timer(0.1, self.calculate_motor_velocity)

        self.action_client = ActionClient(self, FullScan, 'full_scan')

        #self.range_listener  # prevent unused variable warnings
        self.velocity_publisher  # prevent unused variable warnings
        
        self.get_logger().info('Auto Nav Initialized')

    def range_listener(self, range_msg):
        self.distance = range_msg.range * 17150
        self.distance = round(self.distance, 2)

        self.get_logger().info(f'Received Pulse: {range_msg.range}, Calculated Distance: {self.distance} cm')

    def calculate_motor_velocity(self):
        vel_msg = Velocity()

        if self.distance >= 30.0:
            self.linear = 0.5
            self.angular = 0.0
        else:
            self.linear = 0.0
            self.angular = 0.0

        vel_msg.left_vel = self.speed * self.linear - self.differential * self.angular
        vel_msg.right_vel = self.speed * self.linear + self.differential * self.angular

        self.velocity_publisher.publish(vel_msg)

    def send_goal(self, start_angle, end_angle):
        goal_msg = FullScan.Goal()
        goal_msg.start_angle = start_angle
        goal_msg.end_angle = end_angle

        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Feedback: {0}, {0}'.format(result.current_angles, result.current_distances))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Feedback: {0}, {0}'.format(feedback.current_angles, feedback.current_distances))

def main(args=None):
    rclpy.init(args=args)

    auto_nav = Auto_Nav()
    rclpy.spin(auto_nav)

    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
