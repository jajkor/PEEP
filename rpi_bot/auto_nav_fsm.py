import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from rpi_bot_interfaces.msg import Velocity
from rpi_bot_interfaces.action import Scan
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

class Auto_Nav_State(ActionState):
    
    def __init__(self):
        super().__init__(Velocity, 'auto_nav', self.create_goal_handler, None, self.response_handler, self.print_feedback)

    def move_forward(self, blackboard: Blackboard) -> str:
        vel_msg = Velocity()

        vel_msg.left_vel = self.speed * self.linear - self.differential * self.angular
        vel_msg.right_vel = self.speed * self.linear + self.differential * self.angular

        self.velocity_publisher.publish(vel_msg)

    def adjust_orientation(self, blackboard: Blackboard) -> str:
        vel_msg = Velocity()

        vel_msg.left_vel = self.speed * self.linear - self.differential * self.angular
        vel_msg.right_vel = self.speed * self.linear + self.differential * self.angular

        self.velocity_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "MOVING",
        CbState([SUCCEED], move_forward),
        transitions={
            SUCCEED: "SCANNING",
            CANCEL: "outcome4",
            ABORT: "outcome4"
        }
    )
    sm.add_state(
        "SCANNING",
        ScanningState(),
        transitions={
            SUCCEED: "SENDING",
            CANCEL: "outcome4",
            ABORT: "outcome4"
        }
    )
    sm.add_state(
        "ADJUSTING",
        AdjustingState(),
        transitions={
            SUCCEED: "STANDBY",
            CANCEL: "outcome4",
            ABORT: "outcome4"
        }
    )

    # pub FSM info
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # create an initial blackoard
    blackboard = Blackboard()
    blackboard.n = 10

    # execute FSM
    outcome = sm(blackboard)
    print(outcome)

    # shutdown ROS 2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
