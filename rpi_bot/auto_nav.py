import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Range
from rpi_bot_interfaces.msg import Velocity
from rpi_bot_interfaces.srv import Scan

import yasmin
from yasmin import CbState
from yasmin import Blackboard

class Auto_Nav(Node, yasmin.StateMachine):
    def __init__(self):
        Node.__init__(self, 'robot_fsm')
        yasmin.StateMachine.__init__(self, outcomes=['path_clear', 'obstacle_detected', 'scan_complete', 'readjust_complete', 'stream_running', 'stream_interrupted'])
        
        self.obstacle_detected = False
        self.range_publisher_active = False

        self.callback_group = ReentrantCallbackGroup()

        self.linear = 0.0
        self.angular = 0.0
        self.list_angle = None
        self.list_distance = None

        # ROS 2 subscriptions and publishers
        self.range_listener = self.create_subscription(Range, 'range', self.range_callback, 10, callback_group=self.callback_group)
        self.fsm_timer = self.create_timer(0.1, self.run)
        self.velocity_publisher = self.create_publisher(Velocity, 'motor_vel', 10, callback_group=self.callback_group)
        self.vel_timer = self.create_timer(0.1, self.velocity_callback, callback_group=self.callback_group)
        self.scan_client = self.create_client(Scan, 'servo_scan', callback_group=self.callback_group)

        self.add_state(
            'IDLE',
            CbState(['stream_running', 'stream_interrupted'], self.idle),
            transitions={
                'stream_running': 'MOVE',
                'stream_interrupted': 'IDLE'
            }
        )
        self.add_state(
            'MOVE',
            CbState(['obstacle_detected', 'path_clear', 'stream_interrupted'], self.move),
            transitions={
                'obstacle_detected': 'SCAN',
                'path_clear': 'MOVE',
                'stream_interrupted': 'IDLE'
            }
        )
        self.add_state(
            'SCAN',
            CbState(['scan_complete'], self.scan),
            transitions={
                'scan_complete': 'READJUST'
            }
        )
        self.add_state(
            'READJUST',
            CbState(['readjust_complete', 'obstacle_detected'], self.readjust),
            transitions={
                'readjust_complete': 'MOVE',
                'obstacle_detected': 'SCAN'
            }
        )

        self.blackboard = Blackboard()

        self.get_logger().info('Robot FSM Node Initialized')

    def scan_request(self, start_angle, stop_angle):
        self.scan_request = Scan.Request()

        self.scan_request.start_angle = start_angle
        self.scan_request.stop_angle = stop_angle

        if not self.scan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service servo_scan not available, waiting...')
            return None

        self.future = self.scan_client.call_async(self.scan_request)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=10.0)
        if self.future.result() is not None:
            return self.future.result()
        else:
            self.get_logger().error('Service call failed!')
            return None
    
    def range_callback(self, range_msg):
        self.distance = range_msg.range

        if range_msg.range <= 30.0:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

        self.get_logger().info(f'Received Distance: {range_msg.range} cm')

    def velocity_callback(self):
        vel_msg = Velocity()

        #vel_msg.left_vel = self.speed * self.linear - self.differential * self.angular
        #vel_msg.right_vel = self.speed * self.linear + self.differential * self.angular
        vel_msg.left_vel = self.linear
        vel_msg.right_vel = self.angular

        if self.count_subscribers('motor_vel') > 0:
            self.velocity_publisher.publish(vel_msg)
            self.get_logger().info(f'Publishing Velocity: left={vel_msg.left_vel}, right={vel_msg.right_vel}')

    def idle(self, userdata=None):
        time.sleep(0.1)

        if self.count_publishers('range') == 0: # May break if more range publishers are added
            return 'stream_interrupted'
        else:
            return 'stream_running'

    def move(self, userdata=None):
        time.sleep(0.1) # Move or replace with ROS2 create_timer

        self.linear = 0.0
        self.angular = 0.0
        
        if self.obstacle_detected:
            return 'obstacle_detected'
        elif self.count_publishers('range') == 0: # May break if more range publishers are added
            return 'stream_interrupted'
        else:
            self.linear = 60.0
            self.angular = 60.0
            return 'path_clear'

    def scan(self, userdata=None):
        self.response = self.scan_request(40.0, 140.0)

        self.list_angle = self.response.list_angle
        self.list_distance = self.response.list_distance

        return 'scan_complete'

    def readjust(self, userdata=None):
        print('Entering Readjust State')
        time.sleep(2)
        if self.obstacle_detected:
            return 'obstacle_detected'
        else:
            self.get_logger().info(f'Readjust: {self.list_angle}, {self.list_distance}')
            return 'readjust_complete'

    def run(self):
        # Run the FSM logic
        while rclpy.ok():
            new_state_name = self.execute(self.blackboard)  # Run the FSM logic
            if new_state_name:
                self.transition(new_state_name)

def main(args=None):
    rclpy.init(args=args)

    auto_nav = Auto_Nav()
    executor = MultiThreadedExecutor(num_threads=6)

    executor.add_node(auto_nav)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        auto_nav.destroy_node()

if __name__ == '__main__':
    main()
