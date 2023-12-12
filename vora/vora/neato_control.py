# ROS imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odom
from vora_interfaces.msg import VORACommand # type: ignore

import typing
from threading import Thread
from enum import Enum
import time


class State():
    enter: typing.Callable
    step: typing.Callable
    exit: typing.Callable

    def __init__(self, enter: typing.Callable | None = None, step: typing.Callable | None = None, exit: typing.Callable | None = None) -> None:
        self.enter = enter if enter is not None else lambda: None
        self.step = step if step is not None else lambda: None
        self.exit = exit if exit is not None else lambda: None


class neato_control(Node):
    """
    TODO Write Docstrings
    """

    def __init__(self):
        super().__init__('neato_control') # type: ignore

        self.create_subscription(VORACommand, 'vora_command', self.command_callback, 10)
        self.create_subscription(Odom, 'odom', self.odom_callback, 10)
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        self.states: dict[str, State] = {
            "wait": State(self.stop),
            "forward": State(self.forward, self.wait_at_target_time),
            "backward": State(self.backward, self.wait_at_target_time),
            "left": State(self.left, self.wait_at_target_time),
            "right": State(self.right, self.wait_at_target_time),
        }
        self.state: State = self.states["wait"]
        self.state.enter()

        self.odom: Odom
        self.vc_person: str
        self.vc_arg: float

        self.target_time: float

        Thread(target=self.loop).start()

    def setState(self, state: State):
        self.state.exit()
        self.state = state
        self.state.enter()

    def command_callback(self, vc):
        command: str = vc.command.strip()
        if command in self.states:
            print(f"{command}, {vc.person}, {vc.arg}")
            self.vc_person = vc.person
            self.vc_arg = vc.arg
            self.setState(self.states[command])
        else:
            print("Unexpected command recieved")

    def odom_callback(self, odom):
        self.odom = odom

    def stop(self) -> None:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel.publish(msg)

    def forward(self) -> None:
        self.target_time = time.time() + self.vc_arg

        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.cmd_vel.publish(msg)

    def backward(self):
        self.target_time = time.time() + self.vc_arg

        msg = Twist()
        msg.linear.x = -0.2
        msg.angular.z = 0.0
        self.cmd_vel.publish(msg)

    def left(self):
        self.target_time = time.time() + self.vc_arg

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.0
        self.cmd_vel.publish(msg)

    def right(self):
        self.target_time = time.time() + self.vc_arg

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -1.0
        self.cmd_vel.publish(msg)

    def wait_at_target_time(self):
        if time.time() >= self.target_time:
            self.setState(self.states["wait"])        

    def loop(self):
        while True:
            self.state.step()
            time.sleep(0.05)


if __name__ == '__main__':
    node = neato_control()
    node.run() # type: ignore

def main(args=None):
    rclpy.init()
    n = neato_control()
    rclpy.spin(n)
    rclpy.shutdown()