from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rpi_bot',
            executable='talker',
            name='talker',
            emulate_tty='true',
            output='log'
        )
    ])