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
        self.frame = None
        self.model = YOLO("yolov8m.pt")

        self.classified_objects = [39, 41, 64, 67, 73, 76]
        self.classified_objects_names = objHelp.object_sortkey
        self.bridge = CvBridge() # used to convert ROS messages to OpenCV

        self.target_obj = "cup"


        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def process_image(self, msg)->None:
        """ 
        Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing
        """
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


    def go_to_obj(self)->None:
        # TODO Implement this using objHelp
        """
        Determines appropriate linear & angular velocity to go towards specified object
        """
        msg = Twist()
        results = self.model.predict(self.frame, classes=self.classified_objects, verbose=False)
        result_df  = objHelp.convert_to_df(results)

        conf_box_all = {}
        for name in self.classified_objects_names:
            conf_box = objHelp.get_obj_from_df(result_df,name)
            conf_box_all[name] = conf_box

        track_all = {}
        for i,name in enumerate(self.classified_objects_names):
            crnt_conf_box = conf_box_all[name]
            print(name)
            print(crnt_conf_box)
            if len(crnt_conf_box) != 0:
                track_all[name] = objHelp.sorter[i].update(crnt_conf_box)
            else:

                track_all[name] = objHelp.sorter[i].update()

        xy = objHelp.find_obj(self.target_obj,track_all)
        print(xy)
        if xy:
            cv.circle(self.frame,xy,5,(0,0,255),-1)
            x_norm = xy[0]/767 - 0.5
            if x_norm >= -0.2 and x_norm <= 0.2:
                msg.linear.x = 0.2
            elif x_norm < -0.2:
                msg.linear.x = 0.0
                msg.angular.z = 0.8
            elif x_norm > 0.2:
                msg.linear.x = 0.0
                msg.angular.z = -0.8
        else:
            msg.linear.x = 0.0
            msg.angular.x = 0.0

        self.vel_pub.publish(msg)


    def loop_wrapper(self)->None:
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv.namedWindow('video_window')

        # TODO Implement what happens in loop
        self.go_to_obj()
        self.run_loop()
        time.sleep(0.1)

    def loop(self)->None:
        # NOTE: only do cv2.imshow and cv2.waitKey in this function 
        if not self.frame is None:
            waitKey = cv.waitKey(1) & 0xFF

if __name__ == '__main__':
    node = vora("/camera/image_raw")
    node.run() # type: ignore

def main(args=None):
    rclpy.init()
    n = vora("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()