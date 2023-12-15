# ROS imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry as Odom
from sensor_msgs.msg import Image
from vora_interfaces.msg import VORACommand # type: ignore

from .submodules.object_tracking.go_to_obj import retrieve
from .submodules.april_localization import april_tag_command as atcommand

import apriltag
import cv2 as cv
from cv_bridge import CvBridge
import typing
from threading import Thread
from enum import Enum
import time
import math
import numpy as np

class State():
    """
    A class to represent a state in the state machine.

    Attributes:
        enter (typing.Callable): the function to call when entering the state
        step (typing.Callable): the function to call every loop of the state
        exit (typing.Callable): the function to call when exiting the state
    """

    enter: typing.Callable
    step: typing.Callable
    exit: typing.Callable

    def __init__(self, enter: typing.Callable | None = None, step: typing.Callable | None = None, exit: typing.Callable | None = None) -> None:
        """
        Initialize the state. Any of the arguments can be None, in which case the state will do nothing

        Args:
            enter (typing.Callable, optional): the function to call when entering the state
            step (typing.Callable, optional): the function to call every loop of the state
            exit (typing.Callable, optional): the function to call when exiting the state
        """
        
        self.enter = enter if enter is not None else lambda: None
        self.step = step if step is not None else lambda: None
        self.exit = exit if exit is not None else lambda: None

def get_angle(q: Quaternion):
    """
    Get the yaw angle of a quaternion

    Args:
        q (geometry_msgs.msg.Quaternion): the quaternion to get the angle from

    Returns:
        angle (float): the yaw angle of the quaternion in radians
    """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    angle = np.arctan2(siny_cosp, cosy_cosp)

    return angle

class neato_control(Node):
    """
    A node to control the neato robot
    """

    def __init__(self, image_topic):
        """
        Initialize the node

        Args:
            image_topic (str): the topic to subscribe to for images
        """

        super().__init__('neato_control') # type: ignore

        # Start the subscriber to recieve commands from the teleop node
        self.create_subscription(VORACommand, 'vora_command', self.command_callback, 10)
        # Start the subscriber to recieve odometry data from the neato
        self.create_subscription(Odom, 'odom', self.odom_callback, 10)
        # Start the subscriber to recieve images from the camera
        self.create_subscription(Image, image_topic, self.image_callback, 10)
        # Start the publisher to send commands to the neato
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a dictionary of states
        self.states: dict[str, State] = {
            "wait": State(self.stop), # waits for a command
            "forward": State(self.forward, self.wait_at_target_time), # moves forward for a specified amount of time
            "backward": State(self.backward, self.wait_at_target_time), # moves backward for a specified amount of time
            "left": State(self.left, self.wait_at_target_time), # turns left for a specified amount of time
            "right": State(self.right, self.wait_at_target_time), # turns right for a specified amount of time
            "set": State(self.set_home_and_wait), # sets the home position to the current position
            "home": State(None, self.go_home), # navigates to the home position using odometry
            "apriltag": State(self.set_home, self.go_towards_apriltag), # approaches an Apriltag
            "cup": State(None, self.go_towards_cup, self.close_window), # approaches a cup
        }
        # Set the initial state to wait
        self.state: State = self.states["wait"]
        self.state.enter()

        # Initialize variables
        self.odom: Odom
        self.vc_person: str
        self.vc_arg: float
        self.image: np.ndarray = np.zeros((480, 640, 3), np.uint8)

        # Set up the CvBridge for converting ROS images to OpenCV images
        self.bridge = CvBridge() 

        # Set up variables to save data over state loops
        self.target_time: float
        self.home: Point

        # Set up variables for apriltag navigation
        self.apriltag_goal = False
        self.apriltag_begin = True
        self.apriltag_located = False

        # Start the main loop in a new thread
        Thread(target=self.loop).start()

    def command_callback(self, vc):
        """
        Callback for when a command is recieved from the teleop node. Sets the state to the commanded state.

        Args:
            vc (VORACommand): the command recieved
        """

        # Check if the command is in the dictionary of states
        command: str = vc.command.strip()
        if command in self.states:
            # If it is, set the state to the state corresponding to the command
            print(f"{command}, {vc.person}, {vc.arg}")
            self.vc_person = vc.person
            self.vc_arg = vc.arg
            self.setState(self.states[command])
        else:
            # If it isn't, print an error message
            print("Unexpected command recieved")

    def odom_callback(self, odom):
        """
        Callback for when odometry data is recieved from the neato. Saves the odometry data.

        Args:
            odom (Odom): the odometry data recieved
        """

        # Save the odometry data
        self.odom = odom

    def image_callback(self, image):
        """
        Callback for when an image is recieved from the camera. Saves the image.

        Args:
            image (Image): the image recieved
        """

        # Convert the image to a cv2 image and save it
        self.image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

    def setState(self, state: State):
        """
        Sets the state to the given state. Exits the current state and enters the new state.

        Args:
            state (State): the state to set the state machine to
        """
        
        # exit the current state
        self.state.exit()
        # update the state
        self.state = state
        # enter the new state
        self.state.enter()

    def stop(self) -> None:
        """
        Stops the robot        
        """
        
        # Create a message to stop, populate it, and publish it
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel.publish(msg)

    def forward(self) -> None:
        """
        Makes the robot move start moving forward and sets the target time to stop moving
        """

        # Set the target time to the current time plus the command argument
        self.target_time = time.time() + self.vc_arg

        # Create a message to move forward, populate it, and publish it
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.cmd_vel.publish(msg)

    def backward(self):
        """
        Makes the robot move start moving backward and sets the target time to stop moving
        """

        # Set the target time to the current time plus the command argument
        self.target_time = time.time() + self.vc_arg

        # Create a message to move backward, populate it, and publish it
        msg = Twist()
        msg.linear.x = -0.2
        msg.angular.z = 0.0
        self.cmd_vel.publish(msg)

    def left(self):
        """
        Makes the robot start turning left and sets the target time to stop turning
        """
        
        # Set the target time to the current time plus the command argument
        self.target_time = time.time() + self.vc_arg

        # Create a message to turn left, populate it, and publish it
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.0
        self.cmd_vel.publish(msg)

    def right(self):
        """
        Makes the robot start turning right and sets the target time to stop turning
        """
        
        # Set the target time to the current time plus the command argument
        self.target_time = time.time() + self.vc_arg

        # Create a message to turn right, populate it, and publish it
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -1.0
        self.cmd_vel.publish(msg)

    def wait_at_target_time(self):
        """
        Changes the state to wait once the target time is reached
        """

        # Check if the target time has been reached
        if time.time() >= self.target_time:
            # If it has, change the state to wait
            self.setState(self.states["wait"])

    def set_home(self):
        """
        Sets the home position to the current position
        """
        
        # Save the current position as the home position
        self.home = self.odom.pose.pose.position

    def set_home_and_wait(self):
        """
        Sets the home position to the current position and changes the state to wait
        """

        # set the home position
        self.set_home()
        # change the state to wait
        self.setState(self.states["wait"])

    def go_home(self):
        """
        Uses odometry to navigate to the home position
        """

        # set the apriltag variables accordingly
        self.apriltag_goal = False
        self.apriltag_begin = True
        self.apriltag_located = False

        # get the current position and angle
        current_position = self.odom.pose.pose.position
        current_angle = get_angle(self.odom.pose.pose.orientation)

        # calculate the distance to the home position
        delta_x = self.home.x - current_position.x
        delta_y = self.home.y - current_position.y
        distance = (delta_x**2 + delta_y**2)**0.5

        # check if the home position has been reached, and if so, change the state to wait
        if distance < 0.05:
            self.setState(self.states["wait"])
            return

        # calculate the angle to the home position
        target_angle = math.atan2(delta_y, delta_x)
        delta_angle = target_angle - current_angle
        delta_angle = (delta_angle + math.pi) % (2*math.pi) - math.pi

        # calculate the angular and linear velocities. angular velocity is proportional 
        # to the angle to the home position linear velocity is proportional to the 
        # distance to the home position, but is reduced when turning. This is to prevent
        # the robot from driving away from the home position while turning towards it
        angular = np.clip(delta_angle, -1, 1)
        linear = np.clip(distance, -0.3, 0.3) * max(1-abs(angular), 0)

        # create a message to move, populate it, and publish it
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel.publish(msg)

    def close_window(self):
        """
        Closes the video window
        """
        
        # close the video window
        cv.destroyWindow("video_window") 

    def go_towards_apriltag(self):
        """
        Goes towards an Apriltag
        """
        
        # locate the Apriltag
        msg, image_with_april, self.apriltag_goal, self.apriltag_begin, self.apriltag_located = atcommand.locate_apriltag(self.apriltag_goal,self.apriltag_begin,self.apriltag_located, self.image)
        
        # if the Apriltag has been reached, start going towards the cup
        if self.apriltag_goal:
            self.setState(self.states["cup"])
        # otherwise, move towards the Apriltag
        else:
            self.cmd_vel.publish(msg)

        # show the image with the Apriltag
        cv.imshow('video_window',image_with_april)
        cv.waitKey(1)

    def go_towards_cup(self):
        """
        Goes towards a cup
        """
        
        # locate the cup
        msg, image_with_bboxes = retrieve(self.image, "cup")

        # if there is a message to move, publish it
        if msg:
            self.cmd_vel.publish(msg)

        # otherwise, the cup has been reached. Go home
        else:
            print("STOP")
            self.setState(self.states["home"])

        # show the image with the cup
        cv.imshow('video_window', image_with_bboxes) # type: ignore
        cv.waitKey(1)

    def loop(self):
        """
        The loop wrapper. Calls the step function of the current state 20 times per second
        """
        
        # forever 
        while True:
            # run the step function of the current state and wait 0.05 seconds
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