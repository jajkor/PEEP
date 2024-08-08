import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rpi_bot_interfaces.msg import Velocity
from rpi_bot_interfaces.srv import Scan
import yasmin
from yasmin import CbState
from yasmin import Blackboard
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import time

class Auto_Nav(Node, yasmin.StateMachine):
    def __init__(self):
        Node.__init__(self, 'robot_fsm')
        yasmin.StateMachine.__init__(self, outcomes=['path_clear', 'obstacle_detected', 'scan_complete', 'readjust_complete'])
        
        self.distance = None
        self.obstacle_detected = False

        self.callback_group = ReentrantCallbackGroup()

        # ROS 2 subscriptions and publishers
        self.range_listener = self.create_subscription(Range, 'range', self.range_callback, 10, callback_group=self.callback_group)
        self.fsm_timer = self.create_timer(0.1, self.run)
        #self.velocity_publisher = self.create_publisher(Velocity, 'motor_vel', 10)

        self.add_state(
            "MOVE",
            CbState(["obstacle_detected", "path_clear"], self.move),
            transitions={
                "obstacle_detected": "SCAN",
                "path_clear": "MOVE"
            }
        )
        self.add_state(
        "SCAN",
        CbState(["scan_complete"], self.scan),
        transitions={
            "scan_complete": "READJUST"
        }
        )
        self.add_state(
            "READJUST",
            CbState(["readjust_complete", "obstacle_detected"], self.readjust),
            transitions={
                "readjust_complete": "MOVE",
                "obstacle_detected": "SCAN"
            }
        )

        self.blackboard = Blackboard()

        self.get_logger().info('Robot FSM Node Initialized')

    def scan_request(self, start_angle, stop_angle):
        self.scan_request = Scan.Request()

        self.scan_request.start_angle = start_angle
        self.scan_request.stop_angle = stop_angle

        return self.client.call_async(self.scan_request)
    
    def range_callback(self, range_msg):
        self.distance = range_msg.range

        if range_msg.range <= 30.0:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

        #self.get_logger().info(f'Received Distance: {range_msg.range} cm')

    def velocity_callback(self):
        vel_msg = Velocity()

        vel_msg.left_vel = self.speed * self.linear - self.differential * self.angular
        vel_msg.right_vel = self.speed * self.linear + self.differential * self.angular

        self.velocity_publisher.publish(vel_msg)

    def move(self, userdata=None):
        print("Entering Move State")
        time.sleep(2)
        if self.obstacle_detected:
            return 'obstacle_detected'
        else:
            return 'path_clear'

    def scan(self, userdata=None):
        print("Entering Scan State")
        time.sleep(2)
        return 'scan_complete'

    def readjust(self, userdata=None):
        print("Entering Readjust State")
        time.sleep(2)
        if self.obstacle_detected:
            return 'obstacle_detected'
        else:
            return 'readjust_complete'

    def run(self):
        # Run the FSM logic
        while rclpy.ok():
            #rclpy.spin_once(self)  # Process incoming messages
            new_state_name = self.execute(self.blackboard)  # Run the FSM logic
            if new_state_name:
                self.transition(new_state_name)

def main(args=None):
    rclpy.init(args=args)

    auto_nav = Auto_Nav()
    executor = MultiThreadedExecutor(num_threads=4)

    executor.add_node(auto_nav)
    executor.spin()

    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
