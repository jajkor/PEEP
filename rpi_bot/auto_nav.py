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

        self.range_listener  # prevent unused variable warnings
        self.velocity_publisher  # prevent unused variable warnings
        
        self.get_logger().info('Auto Nav Initialized')

    def range_listener(self, range_msg):
        self.distance = range_msg.range * 17150
        self.distance = round(self.distance, 2)

        self.get_logger().info(f'Received Pulse: {range_msg.range}, Calculated Distance: {self.distance} cm')

    def calculate_wheel_velocity(self):
        #left_temp = self.left_vel
        #right_temp = self.right_vel
        msg = Velocity()
        left_temp = self.speed * msg.linear.x - self.differential * msg.angular.z
        right_temp = self.speed * msg.linear.x + self.differential * msg.angular.z

        msg.left_vel = left_temp
        msg.right_vel = right_temp

        self.velocity_publisher.publish(msg)
            #self.get_logger().info(f'Publishing Velocity: left={left_temp}, right={right_temp}')

        
        #self.get_logger().info(f'Setting Motors: left_vel={left_temp}, right_vel={right_temp}')
        #self.velocity_publisher.publish()

        #if (left_temp != self.left_vel) and (right_temp != self.right_vel):
            #self.get_logger().info(f'Setting motors: left_vel={left_temp}, right_vel={right_temp}')
            #self.left_vel = left_temp
            #self.right_vel = right_temp
            #self.velocity_publisher.publish()

def main(args=None):
    rclpy.init(args=args)

    hcs04_subscriber = HCS04_Subscriber()
    rclpy.spin(hcs04_subscriber)

    hcs04_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
