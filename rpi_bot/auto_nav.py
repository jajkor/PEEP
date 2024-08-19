import time

import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Range
from rpi_bot_interfaces.srv import Velocity
from rpi_bot_interfaces.srv import Scan
from rpi_bot.utils import clamp

import yasmin
from yasmin import CbState
from yasmin import Blackboard

class Auto_Nav(Node, yasmin.StateMachine):
    THRESHHOLD_DISTANCE = 34.48 # 1 Foot (30.48) + Distance to Sensor
    MAX_SPEED = 80.0
    ADJUST_SPEED = MAX_SPEED / 2.0
    MAX_DISTANCE = 1200.0

    def __init__(self):
        Node.__init__(self, 'robot_fsm')
        yasmin.StateMachine.__init__(self, outcomes=['path_clear', 'obstacle_detected', 'scan_complete', 'readjust_complete', 'stream_running', 'stream_interrupted'])
        
        self.obstacle_detected = False

        self.callback_group = ReentrantCallbackGroup()
        self.srv_callback_group = ReentrantCallbackGroup()

        self.velocity = Auto_Nav.MAX_SPEED

        # ROS 2 subscriptions and publishers
        self.range_listener = self.create_subscription(Range, 'range', self.range_callback, qos.qos_profile_sensor_data, callback_group=self.callback_group)

        self.scan_client = self.create_client(Scan, 'servo_scan', callback_group=self.srv_callback_group)
        self.scan_request = Scan.Request()
        self.velocity_client = self.create_client(Velocity, 'set_velocity', callback_group=self.srv_callback_group)
        self.velocity_request = Velocity.Request()

        self.fsm_timer = self.create_timer(0.1, self.run)

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
            CbState(['readjust_complete', 'readjust_complete', 'obstacle_detected'], self.readjust),
            transitions={
                'readjust_complete': 'MOVE',
                'obstacle_detected': 'SCAN'
            }
        )

        self.blackboard = Blackboard()
        self.get_logger().info('Robot FSM Node Initialized')

    def send_scan_request(self, start_angle, stop_angle):
        self.scan_request.start_angle = start_angle
        self.scan_request.stop_angle = stop_angle

        self.scan_future = self.scan_client.call_async(self.scan_request)
        self.executor.spin_until_future_complete(self.scan_future, timeout_sec=10.0)
        return self.scan_future.result()
    
    def send_velocity_request(self, left_velocity, right_velocity):
        self.velocity_request.left_velocity = left_velocity
        self.velocity_request.right_velocity = right_velocity

        self.velocity_future = self.velocity_client.call_async(self.velocity_request)
        self.executor.spin_until_future_complete(self.velocity_future, timeout_sec=1.0)
        return self.velocity_future.result()

    def range_callback(self, range_msg):
        self.distance = clamp(range_msg.range, 0, Auto_Nav.MAX_DISTANCE)

        self.velocity = clamp(((self.distance * 1) - (Auto_Nav.THRESHHOLD_DISTANCE)), 0.0, Auto_Nav.MAX_SPEED)

        if range_msg.range <= Auto_Nav.THRESHHOLD_DISTANCE:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

        #self.get_logger().info(f'Received Distance: {range_msg.range} cm')

    def idle(self, userdata=None):
        time.sleep(0.1)

        if self.count_publishers('range') == 0: # May break if more range publishers are added
            return 'stream_interrupted'
        else:
            return 'stream_running'

    def move(self, userdata=None):  
        if self.obstacle_detected:
            self.send_velocity_request(0.0, 0.0)
            return 'obstacle_detected'
        elif self.count_publishers('range') == 0: # May break if more range publishers are added
            self.send_velocity_request(0.0, 0.0)
            return 'stream_interrupted'
        else:
            self.send_velocity_request(self.velocity, self.velocity)
            return 'path_clear'

    def scan(self, userdata=None):
        response = self.send_scan_request(0.0, 190.0)

        self.dict = {response.list_angle[i]: response.list_distance[i] for i in range(len(response.list_angle))}

        return 'scan_complete'

    def readjust(self, userdata=None):
        print('Entering Readjust State')
        
        temp_k = 180.0
        temp_v = Auto_Nav.THRESHHOLD_DISTANCE
        for key, value in self.dict.items():
            if temp_v < value and value < Auto_Nav.MAX_DISTANCE:
                temp_k = key
                temp_v = value

        self.get_logger().info(f'Angle: {temp_k}, Distance: {temp_v}')

        while (self.distance <= temp_v) and (self.distance <= Auto_Nav.THRESHHOLD_DISTANCE):
            self.get_logger().info(f'Distance: {temp_v}')
            if temp_k == 180.0:
                self.send_velocity_request(-Auto_Nav.ADJUST_SPEED, -Auto_Nav.ADJUST_SPEED)
                time.sleep(1)
                self.send_velocity_request(0.0, 0.0)
                return 'obstacle_detected'
            elif temp_k >= 90.0 and temp_k < 180.0:
                self.send_velocity_request(Auto_Nav.ADJUST_SPEED, -Auto_Nav.ADJUST_SPEED)
            elif temp_k < 90.0:
                self.send_velocity_request(-Auto_Nav.ADJUST_SPEED , Auto_Nav.ADJUST_SPEED)

        self.send_velocity_request(0.0, 0.0)

        #time.sleep(2)
        if self.obstacle_detected:
            return 'obstacle_detected'
        else:
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
