import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist')

    teleop_twist_joy_config = os.path.join(
        get_package_share_directory('rpi_bot'),
        'config',
        'xbox.config.yaml'
    )
    print(f"Config File: {teleop_twist_joy_config}")

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='cmd_vel'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        launch.actions.DeclareLaunchArgument('publish_stamped_twist', default_value='false'),
           
        launch_ros.actions.Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node',
            parameters=[{'dev': joy_dev, 'deadzone': 0.3, 'autorepeat_rate': 20.0}],
        ),
        launch_ros.actions.Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[teleop_twist_joy_config, {'publish_stamped_twist': publish_stamped_twist}],
            remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))},
        ),
        launch_ros.actions.Node(
            package='rpi_bot',
            executable='sub_hcs04_node',
            name='sub_hcs04',
        ),
        launch_ros.actions.Node(
            package='rpi_bot',
            executable='sub_usb_cam_node',
            name='sub_usb_cam',
        ),
    ])
