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
                'enb_pin': 26,
                'speed': 75,
                'differential': 75,
            }]
        ),
        launch_ros.actions.Node(
            package='rpi_bot',
            executable='hcs04_driver_node',
            name='hcs04_driver',
            parameters=[{
                'trig_pin': 15,
                'echo_pin': 14
            }]
        ),
        launch_ros.actions.Node( # HC-S04 Servo
            package='rpi_bot',
            executable='sub_sg90_node',
            name='sub_hcs04_sg90',
            parameters=[{
                'pwm_channel': 0,
                'left_btn': 4,
                'right_btn': 5,
                'reverse': True,
                'axes': False,
                'axes_btn': -1
            }]
        ),
        launch_ros.actions.Node( # Camera Pan Servo (D-Pad = 6, Joy = 3)
            package='rpi_bot',
            executable='sub_sg90_node',
            name='sub_pan_sg90',
            parameters=[{
                'pwm_channel': 2,
                'left_btn': -1,
                'right_btn': -1,
                'reverse': False,
                'axes': True,
                'axes_btn': 3
            }]
        ),
        launch_ros.actions.Node( # Camera Tilt Servo (D-Pad = 7, Joy = 4)
            package='rpi_bot',
            executable='sub_sg90_node',
            name='sub_tilt_sg90',
            parameters=[{
                'pwm_channel': 1,
                'left_btn': -1,
                'right_btn': -1,
                'reverse': False,
                'axes': True,
                'axes_btn': 4
            }]
        ),
    ])