import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
class HCS04_Subscriber(Node):

    def __init__(self):
        super().__init__('auto_nav')

        self.speed = 75
        self.differential = 75

        self.range_listener = self.create_subscription(Range, 'range', self.range_listener, 10)
        
        self.twist_listener = self.create_subscription(Twist, 'cmd_vel', self.calculate_wheel_velocity_callback, 10)

        self.range_listener  # prevent unused variable warnings
        self.twist_listener  # prevent unused variable warnings
        
        self.get_logger().info('Auto Nav Initialized')

    def range_listener(self, range_msg):
        distance = range_msg.range * 17150
        distance = round(distance, 2)

        self.get_logger().info(f'Received Pulse Duration: {range_msg.range}, Calculated Distance: {distance} cm')

    def calculate_wheel_velocity_callback(self, msg):
        #left_temp = self.left_vel
        #right_temp = self.right_vel

        left_temp = self.speed * msg.linear.x - self.differential * msg.angular.z
        right_temp = self.speed * msg.linear.x + self.differential * msg.angular.z
        
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
