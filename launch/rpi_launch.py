import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rpi_bot',
            executable='cmd_to_pwm_node',
            name='cmd_to_pwm',
            parameters=[{
                'ena_pin': 25,
                'in1_pin': 24,
                'in2_pin': 23,
                'in3_pin': 22,
                'in4_pin': 27,
                'enb_pin': 17,
                'speed': 75,
                'differential': 75
            }]
        ),
        launch_ros.actions.Node(
            package='rpi_bot',
            executable='pub_hcs04_node',
            name='pub_hcs04',
            parameters=[{
                'trig_pin': 18,
                'echo_pin': 24
            }]
        ),
        launch_ros.actions.Node(
            package='rpi_bot',
            executable='sub_sg90_node',
            name='sub_sg90',
            parameters=[{
                'pwm_pin': 14
            }]
        )
    ])