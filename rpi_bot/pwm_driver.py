import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rpi_bot.rpi_interface import RPi_Motors
from rpi_bot_interfaces.srv import Velocity

class PWM_Driver(Node):

    def __init__(self):
        super().__init__('set_vel')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ena_pin', rclpy.Parameter.Type.INTEGER),
                ('in1_pin', rclpy.Parameter.Type.INTEGER),
                ('in2_pin', rclpy.Parameter.Type.INTEGER),
                ('in3_pin', rclpy.Parameter.Type.INTEGER),
                ('in4_pin', rclpy.Parameter.Type.INTEGER),
                ('enb_pin', rclpy.Parameter.Type.INTEGER),
            ],
        )

        self.motors = RPi_Motors(
            self.get_parameter('ena_pin').get_parameter_value().integer_value,
            self.get_parameter('in1_pin').get_parameter_value().integer_value,
            self.get_parameter('in2_pin').get_parameter_value().integer_value,
            self.get_parameter('in3_pin').get_parameter_value().integer_value,
            self.get_parameter('in4_pin').get_parameter_value().integer_value,
            self.get_parameter('enb_pin').get_parameter_value().integer_value
        )

        self.left_velocity = 0.0
        self.right_velocity = 0.0

        self.velocity_callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(Velocity, 'set_velocity', self.velocity_callback, callback_group=self.velocity_callback_group)
        self.get_logger().info('PWM Driver Initialized')

    def velocity_callback(self, request, response):
        if (self.left_velocity != request.left_velocity) and (self.right_velocity != request.right_velocity):
            self.left_velocity = request.left_velocity
            self.right_velocity = request.right_velocity
            self.motors.set_motors(request.left_velocity, request.right_velocity)
            self.get_logger().info(f"Left Velocity: {request.left_velocity}, Right Velocity: {request.right_velocity}")
        
        response.left_velocity = request.left_velocity
        response.right_velocity = request.right_velocity

        return response

def main(args=None):
    rclpy.init(args=args)

    velocity = PWM_Driver()
    velocity_executor = MultiThreadedExecutor()
    velocity_executor.add_node(velocity)

    try:
        velocity_executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        velocity.destroy_node()


if __name__ == '__main__':
    main()