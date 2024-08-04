import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rpi_bot',
            executable='pwm_driver_node',
            name='pwm_driver',
            parameters=[{
                'ena_pin': 6,
                'in1_pin': 12,
                'in2_pin': 13,
                'in3_pin': 20,
                'in4_pin': 21,
                'enb_pin': 26
            }]
        ),
        launch_ros.actions.Node(
            package='rpi_bot',
            executable='hcs04_driver_node',
            name='hcs04_scanner',
            parameters=[{
                'trig_pin': 15,
                'echo_pin': 14
            }]
        ),
        launch_ros.actions.Node( # HC-S04 Servo
            package='rpi_bot',
            executable='servo_scan_service',
            name='servo_scan',
            parameters=[{
                'pwm_channel': 0,
                'starting_angle': 90
            }]
        ),
    ])