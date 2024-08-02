import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from rpi_bot_interfaces.msg import Velocity

class HCS04_Subscriber(Node):

    def __init__(self):
        super().__init__('auto_nav')

        self.speed = 75
        self.differential = 75

        self.range_listener = self.create_subscription(Range, 'range', self.range_listener, 10)
        self.velocity_publisher = self.create_publisher(Velocity, 'motor_vel', 10)
        self.timer = self.create_timer(0.1, self.calculate_wheel_velocity)

        self.range_listener  # prevent unused variable warnings
        self.velocity_publisher  # prevent unused variable warnings
        
        self.get_logger().info('Auto Nav Initialized')

    def range_listener(self, range_msg):
        self.distance = range_msg.range * 17150
        self.distance = round(self.distance, 2)

        self.get_logger().info(f'Received Pulse: {range_msg.range}, Calculated Distance: {self.distance} cm')

    def calculate_wheel_velocity(self):
        vel_msg = Velocity()
        
        left_temp = self.speed * 0.5 - self.differential * 0.5
        right_temp = self.speed * 0.5 + self.differential * 0.5

        vel_msg.left_vel = left_temp
        vel_msg.right_vel = right_temp

        self.velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)

    hcs04_subscriber = HCS04_Subscriber()
    rclpy.spin(hcs04_subscriber)

    hcs04_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
