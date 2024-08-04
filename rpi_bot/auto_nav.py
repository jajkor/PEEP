import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rpi_bot_interfaces.msg import Velocity
from rpi_bot_interfaces.srv import Scan
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub
import time

class Auto_Nav(Node):
    def __init__(self):
        super().__init__('robot_fsm')

        # Initialize FSM and add transitions
        self.fsm = StateMachine(outcomes=['obstacle_detected', 'scan_complete', 'readjust_complete'])
        self.fsm.add_state(
            "MOVE",
            CbState(["obstacle_detected"], self.move),
            transitions={
                "obstacle_detected": "SCAN"
            }
        )
        self.fsm.add_state(
            "SCAN",
            CbState(["scan_complete"], self.scan),
            transitions={
                "scan_complete": "READJUST"
            }
        )
        self.fsm.add_state(
            "READJUST",
            CbState(["readjust_complete"], self.readjust),
            transitions={
                "readjust_complete": "MOVE"
            }
        )

        # ROS 2 subscriptions and publishers
        self.range_listener = self.create_subscription(Range, 'range', self.range_callback, 10)
        self.velocity_publisher = self.create_publisher(Velocity, 'motor_vel', 10)
        
        self.get_logger().info('Robot FSM Node Initialized')

    def move(self):
        self.get_logger().info("Entering Move State")
        time.sleep(3)
        return 'obstacle_detected'

    def scan(self):
        self.get_logger().info("Entering Scan State")
        time.sleep(3)
        return 'scan_complete'

    def readjust(self):
        self.get_logger().info("Entering Readjust State")
        time.sleep(3)
        return 'readjust_complete'

    def range_callback(self, range_msg):
        self.get_logger().info(f'Received Distance: {range_msg.range} cm')

        if range_msg.range <= 30:
            #self.send_request(0.0, 180.0)
            print()

    def send_request(self, start_angle, stop_angle):
        self.scan_request = Scan.Request()

        self.scan_request.start_angle = start_angle
        self.scan_request.stop_angle = stop_angle

        return self.client.call_async(self.scan_request)

    def calculate_motor_velocity(self):
        vel_msg = Velocity()

        vel_msg.left_vel = self.speed * self.linear - self.differential * self.angular
        vel_msg.right_vel = self.speed * self.linear + self.differential * self.angular

        self.velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)

    auto_nav = Auto_Nav()
    rclpy.spin(auto_nav)

    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
