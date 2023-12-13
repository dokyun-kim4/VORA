# ROS imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry as Odom
from sensor_msgs.msg import Image
from vora_interfaces.msg import VORACommand # type: ignore

from .submodules.object_tracking.go_to_obj import retrieve

import cv2 as cv
from cv_bridge import CvBridge
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

    def __init__(self, image_topic):
        super().__init__('neato_control') # type: ignore

        self.create_subscription(VORACommand, 'vora_command', self.command_callback, 10)
        self.create_subscription(Odom, 'odom', self.odom_callback, 10)
        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        self.states: dict[str, State] = {
            "wait": State(self.stop),
            "forward": State(self.forward, self.wait_at_target_time),
            "backward": State(self.backward, self.wait_at_target_time),
            "left": State(self.left, self.wait_at_target_time),
            "right": State(self.right, self.wait_at_target_time),
            "set": State(self.set_home),
            "home": State(None, self.go_home),
            "cup": State(None, self.go_towards_cup, self.close_window),
            "bottle": State(None, self.go_towards_bottle, self.close_window),
        }
        self.state: State = self.states["wait"]
        self.state.enter()

        self.odom: Odom
        self.vc_person: str
        self.vc_arg: float
        self.image: np.ndarray = np.zeros((480, 640, 3), np.uint8)

        self.bridge = CvBridge() 

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

    def image_callback(self, image):
        self.image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

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

        angular = np.clip(delta_angle, -1, 1)

        print("distance:", distance, (1-abs(angular)))

        linear = np.clip(distance, -0.3, 0.3) * max(1-abs(angular), 0)

        print("commands:", linear, angular)

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel.publish(msg)

    def close_window(self):
        cv.destroyWindow("video_window") 

    def go_towards_cup(self):
        msg, image_with_bboxes = retrieve(self.image, "cup")
        self.cmd_vel.publish(msg)
        cv.imshow('video_window', image_with_bboxes) # type: ignore
        cv.waitKey(1)

    def go_towards_bottle(self):
        msg, image_with_bboxes = retrieve(self.image, "bottle")
        self.cmd_vel.publish(msg)
        cv.imshow('video_window', image_with_bboxes) # type: ignore
        cv.waitKey(1)

    def loop(self):
        while True:
            self.state.step()
            time.sleep(0.05)


if __name__ == '__main__':
    node = neato_control("/camera/image_raw")
    node.run() # type: ignore

def main(args=None):
    rclpy.init()
    n = neato_control("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()