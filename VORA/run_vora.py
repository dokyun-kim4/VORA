# ROS imports
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry as Odom
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump

# Computer vision imports
import cv2 as cv
from ultralytics import YOLO
from object_tracking.sort import Sort
from object_tracking import helpers as objHelp

# Voice-related imports
from voice_detection import helpers as voiceHelp

# Misc.
import time
from threading import Thread
import numpy as np
from typing import Literal

class vora(Node):
    """
    TODO Write Docstrings
    """
    def __init__(self,image_topic):
        super.__init__('run_vora') # type: ignore
        self.crnt_frame = None
        self.prev_frame = None

        self.bridge = CvBridge() # used to convert ROS messages to OpenCV



        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def process_image(self, msg):
        """ 
        Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing
        """
        if self.crnt_frame is not None:
            self.prev_frame = self.crnt_frame.copy() # type: ignore
        self.crnt_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


    def goToObj(self):
        # TODO Implement this using objHelp
        pass



    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv.namedWindow('video_window')

        # TODO Implement what happens in loop

        self.run_loop()
        time.sleep(0.1)



    def run_loop(self):
        # NOTE: only do cv2.imshow and cv2.waitKey in this function 
        if not self.crnt_frame is None:
            waitKey = cv.waitKey(1) & 0xFF

if __name__ == '__main__':
    node = vora("/camera/image_raw")
    node.run() # type: ignore