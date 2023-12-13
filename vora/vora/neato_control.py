# ROS imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry as Odom
from vora_interfaces.msg import VORACommand # type: ignore

import typing
from threading import Thread
from enum import Enum
import time
import math
import numpy as np

class State():
    enter: typing.Callable
    step: typing.Callable
    exit: typing.Callable

    def __init__(self, enter: typing.Callable | None = None, step: typing.Callable | None = None, exit: typing.Callable | None = None) -> None:
        self.enter = enter if enter is not None else lambda: None
        self.step = step if step is not None else lambda: None
        self.exit = exit if exit is not None else lambda: None

def get_angle(q: Quaternion):
    """
    Get the yaw angle of a quaternion

    Args:
        q (geometry_msgs.msg.Quaternion): the quaternion to get the angle from

    Returns:
        angle (float): the yaw angle of the quaternion in degrees
    """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    angle = np.arctan2(siny_cosp, cosy_cosp)

    return angle

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
            "set_home": State(self.set_home),
            "home": State(self.debug_go_home, self.go_home),
        }
        self.state: State = self.states["wait"]
        self.state.enter()

        self.odom: Odom
        self.vc_person: str
        self.vc_arg: float

        self.target_time: float
        self.home: Point

        Thread(target=self.loop).start()

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

    def setState(self, state: State):
        self.state.exit()
        self.state = state
        self.state.enter()

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

    def set_home(self):
        self.home = self.odom.pose.pose.position
        self.setState(self.states["wait"])

    def debug_go_home(self):
        print("home:", self.home.x, self.home.y)
        print("current:", self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, get_angle(self.odom.pose.pose.orientation))

    def go_home(self):
        current_position = self.odom.pose.pose.position
        current_angle = get_angle(self.odom.pose.pose.orientation)

        delta_x = self.home.x - current_position.x
        delta_y = self.home.y - current_position.y

        distance = (delta_x**2 + delta_y**2)**0.5
        if distance < 0.05:
            self.setState(self.states["wait"])
            return

        target_angle = math.atan2(delta_y, delta_x)
        delta_angle = target_angle - current_angle
        delta_angle = (delta_angle + math.pi) % (2*math.pi) - math.pi
        print("state:", round(delta_x, 2), round(delta_y, 2), round(delta_angle, 2))

        angular = np.clip(delta_angle, -0.5, 0.5)

        print("distance:", distance, (1-abs(angular)))

        linear = np.clip(distance, -0.3, 0.3) * max(1-abs(2*angular), 0)

        print("commands:", linear, angular)

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel.publish(msg)

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