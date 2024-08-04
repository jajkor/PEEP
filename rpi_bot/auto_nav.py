import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rpi_bot_interfaces.msg import Velocity
from rpi_bot_interfaces.srv import Scan

class Auto_Nav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        self.speed = 75
        self.differential = 75
        self.linear = 0
        self.angular = 0
        self.distance = 0

        #self.velocity_publisher = self.create_publisher(Velocity, 'motor_vel', 10)
        #self.velocity_timer = self.create_timer(0.1, self.calculate_motor_velocity)

        self.range_listener = self.create_subscription(Range, 'range', self.range_listener, 10)
        self.client = self.create_client(Scan, 'servo_scan')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.range_listener  # prevent unused variable warnings
        self.velocity_publisher  # prevent unused variable warnings
        self.interrupt_publisher
        
        self.get_logger().info('Auto Nav Initialized')

    def range_listener(self, range_msg):
        self.distance = range_msg.range * 17150
        self.distance = round(self.distance, 2)
        self.get_logger().info(f'Received Pulse: {range_msg.range}, Calculated Distance: {self.distance} cm')

        if self.distance <= 30:
            self.send_request(0.0, 180.0)

    def send_request(self, start_angle, stop_angle):
        self.scan_request.start_angle = start_angle
        self.scan_request.stop_angle = stop_angle

        return self.cli.call_async(self.scan_request)

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
