import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rpi_bot_interfaces.msg import Velocity
from rpi_bot_interfaces.srv import Scan
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub
import time

class Auto_Nav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        # Initialize FSM states
        self.move_state = State('Move', on_enter=self.move)
        self.scan_state = State('Scan', on_enter=self.scan)
        self.readjust_state = State('Readjust', on_enter=self.readjust)

        # Initialize FSM and add transitions
        self.fsm = StateMachine(self.move_state)
        self.fsm.add_transition(self.move_state, self.scan_state, trigger='obstacle_detected')
        self.fsm.add_transition(self.scan_state, self.readjust_state, trigger='scan_complete')
        self.fsm.add_transition(self.readjust_state, self.move_state, trigger='readjust_complete')

        # ROS 2 subscriptions and publishers
        self.range_listener = self.create_subscription(Range, 'range', self.range_callback, 10)
        #self.velocity_publisher = self.create_publisher(Velocity, 'motor_vel', 10)

        self.client = self.create_client(Scan, 'servo_scan')     
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.get_logger().info('Auto Nav Initialized')

    def move(self):
        self.get_logger().info('Moving forward')
        #twist = Twist()
        #twist.linear.x = 0.5
        #self.cmd_vel_publisher.publish(twist)

    def scan(self):
        self.get_logger().info('Scanning for obstacles')
        # Implement scanning logic
        time.sleep(3)
        # Transition to 'Readjust' state when scanning is complete
        self.fsm.scan_complete()

    def readjust(self):
        self.get_logger().info('Readjusting path')
        # Implement path readjustment logic
        time.sleep(3)
        # Transition to 'Move' state when readjustment is complete
        self.fsm.readjust_complete()

    def range_callback(self, range_msg):
        self.get_logger().info(f'Received Distance: {range_msg.range} cm')

        if self.distance <= 30:
            self.send_request(0.0, 180.0)
            self.fsm.obstacle_detected()

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

# create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "MOVING",
        MovingState(),
        transitions={
            "outcome1": "SCANNING",
            "outcome2": "MOVING"
        }
    )
    sm.add_state(
        "SCANNING",
        ScanningState(),
        transitions={
            "outcome3": "TURNING"
        }
    )
    sm.add_state(
        "TURNING",
        TurningState(),
        transitions={
            "outcome3": "MOVING"
        }
    )

    # pub FSM info
    YasminViewerPub("yasmin_demo", sm)

    # execute FSM
    outcome = sm()
    print(outcome)

    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
