import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[{
            'axis_linear.x': 1,  # Set the appropriate axis for linear x
            'axis_angular.yaw': 0,  # Set the appropriate axis for angular yaw
            'scale_linear.x': 1.0,  # Set the scale for linear x
            'scale_angular.yaw': 1.0,  # Set the scale for angular yaw
        }]
    )

    return LaunchDescription([

        teleop_twist_joy_node
    ])