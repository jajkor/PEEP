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

        # ROS 2 subscriptions and publishers
        self.range_listener = self.create_subscription(Range, 'range', self.range_callback, 10)
        self.velocity_publisher = self.create_publisher(Velocity, 'motor_vel', 10)

        self.get_logger().info('Robot FSM Node Initialized')

    def scan_request(self, start_angle, stop_angle):
        self.scan_request = Scan.Request()

        self.scan_request.start_angle = start_angle
        self.scan_request.stop_angle = stop_angle

        return self.client.call_async(self.scan_request)
    
    def range_callback(self, range_msg, blackboard: Blackboard):
        blackboard.distance = range_msg.range
        self.get_logger().info(f'Received Distance: {range_msg.range} cm')

    def velocity_callback(self):
        vel_msg = Velocity()

        vel_msg.left_vel = self.speed * self.linear - self.differential * self.angular
        vel_msg.right_vel = self.speed * self.linear + self.differential * self.angular

        self.velocity_publisher.publish(vel_msg)

def move(blackboard: Blackboard):
    print("Entering Move State")
    time.sleep(3)
    print(blackboard.distance)
    if False:
        return 'obstacle_detected'
    else:
        return 'path_clear'

def scan(self):
    print("Entering Scan State")
    time.sleep(3)
    return 'scan_complete'

def readjust(self):
    print("Entering Readjust State")
    time.sleep(3)
    return 'readjust_complete'

def main(args=None):
    rclpy.init(args=args)

    auto_nav = Auto_Nav()

    # Initialize FSM and add transitions
    sm = StateMachine(outcomes=['path_clear', 'obstacle_detected', 'scan_complete', 'readjust_complete'])
    sm.add_state(
        "MOVE",
        CbState(["obstacle_detected", "path_clear"], move),
        transitions={
            "obstacle_detected": "SCAN",
            "path_clear": "MOVE"
        }
    )
    sm.add_state(
        "SCAN",
        CbState(["scan_complete"], scan),
        transitions={
            "scan_complete": "READJUST"
        }
    )
    sm.add_state(
        "READJUST",
        CbState(["readjust_complete", "object_detected"], readjust),
        transitions={
            "readjust_complete": "MOVE",
            "object_detected": "SCAN"
        }
    )

    blackboard = Blackboard()
    blackboard.distance = None
    outcome = sm(blackboard)
    print(outcome)

    rclpy.spin(auto_nav)

    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
