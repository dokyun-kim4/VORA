# ROS imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from vora_interfaces.msg import VORACommand # type: ignore

from threading import Thread
from enum import Enum
import time

class State(Enum):
    WAIT = 0
    FORWARD = 1

command2State: dict[str, State] = {
    "wait": State.WAIT,
    "forward": State.FORWARD,
}

class neato_control(Node):
    """
    TODO Write Docstrings
    """
    def __init__(self):
        super().__init__('neato_control') # type: ignore

        self.create_subscription(VORACommand, 'vora_command', self.command_callback, 10)
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        self.state: State = State.WAIT

        Thread(target=self.loop).start()

    def command_callback(self, vc):
        self.state = command2State[vc.command]

    def stop(self) -> None:
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.cmd_vel.publish(msg)

    def loop(self):
        while True:

            match self.state:
                case State.WAIT:
                    pass
                case State.FORWARD:
                    pass

            time.sleep(0.05)


if __name__ == '__main__':
    node = neato_control()
    node.run() # type: ignore

def main(args=None):
    rclpy.init()
    n = neato_control()
    rclpy.spin(n)
    rclpy.shutdown()