# ROS imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from threading import Thread
import cv2
import time
import apriltag
import numpy as np
from .submodules.april_localization import april_tag_command as atcommand

class aprilTagTest(Node):
    """
    TODO Write Docstrings
    """
    def __init__(self, image_topic):
        super().__init__('apriltag_test') # type: ignore

        self.apriltag_goal = False
        self.apriltag_begin = True
        self.apriltag_located = False

        self.image: np.ndarray = np.zeros((480, 640, 3), np.uint8)
        self.bridge = CvBridge() 

        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        thread = Thread(target=self.loop_wrapper)
        thread.start()
    
    def image_callback(self, image):
        self.image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")


    def loop_wrapper(self)->None:
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')

        # TODO Implement what happens in loop
        while True:
            msg, self.image, self.apriltag_goal, self.apriltag_begin, self.apriltag_located = atcommand.locate_apriltag(self.apriltag_goal, self.apriltag_begin, self.apriltag_located, self.image)
            self.vel_pub.publish(msg)
            self.loop()
            time.sleep(0.1)
        

    def loop(self)->None:
        # NOTE: only do cv2.imshow and cv2.waitKey in this function 
        if not self.image is None:
            cv2.imshow('video_window', self.image)
            cv2.waitKey(5)


if __name__ == '__main__':
    node = aprilTagTest("/camera/image_raw")
    node.run() # type: ignore

def main(args=None):
    rclpy.init()
    n = aprilTagTest("/camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()