import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rpi_bot',
            executable='cmd_to_pwm_node',
            name='cmd_to_pwm',
            parameters=[{
                'ena_pin': 6,
                'in1_pin': 12,
                'in2_pin': 13,
                'in3_pin': 20,
                'in4_pin': 21,
                'enb_pin': 26,
                'speed': 75,
                'differential': 75,
            }]
        ),
        launch_ros.actions.Node(
            package='rpi_bot',
            executable='pub_hcs04_node',
            name='pub_hcs04',
            parameters=[{
                'trig_pin': 3,
                'echo_pin': 2
            }]
        ),
        launch_ros.actions.Node(
            package='rpi_bot',
            executable='sg90_node',
            name='sg90',
            parameters=[{
                #'pwm_pin': 4
                'pwm_channel': 0,
            }]
        )
    ])