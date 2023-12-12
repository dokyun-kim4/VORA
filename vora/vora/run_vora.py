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

# AprilTag Imports
import apriltag
from april_localization import april_tag_helpers as athelp

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

        # AprilTag Variables
        self.apriltag_begin = False
        self.apriltag_located = False
        self.apriltag_goal = False


        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def process_image(self, msg)->None:
        """ 
        Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing
        """
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    
    def locate_apriltag(self, msg, tag_family="tag36h11"):
        #TODO: Implement AprilTag locating and pathing
        #1. Rotate until AprilTag is located x
        #2. Check angle of AprilTag - if it is suitable move straight ahead.
        #2  If angle is not suitable, then rotate and drive in direction to get better angle.
        #3 Re-check Apriltag angle and repeat above steps until tag is centered with Neato
        #4 Once tag is centered, move towards it until within specified distance
        #5 Switch over to object detect

        if self.apriltag_goal is True: #If tag has already been reached, skip
            return
        if self.apriltag_begin is False: #If tag detection has not begun yet, skip
            return
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(self.frame)
        if self.apriltag_located is False: #while tag has not been located
            if len(results) == 0: #if no tag, turn in a circle until tag detected
                msg.angular.z = -0.2
                msg.linear.x = 0
            elif abs(athelp.create_coord_pair(results)[0]) < 50: #if there is a tag, turn until center of frame.
                msg.angular.z = 0
                msg.linear.x = 0
                self.apriltag_located = True
        if self.apriltag_located is True:
            self.frame, tag_angle = athelp.draw_april_angle(self.frame, results)
            if abs(tag_angle) > 5:
                self.correct_for_angle(tag_angle, msg) #TODO: Create this function
                msg.linear.x = 0
                msg.angular.z = 0
                self.apriltag_located = False
                return
            msg.linear.x = 2
            self.apriltag_goal = True

    def correct_for_angle(self, tag_angle, msg):
        # If neato needs to move to the right
        if tag_angle > 0:
            msg.angular.z = .5 * tag_angle
            time.sleep(.5 * tag_angle)
            msg.linear.x = 0.5 * tag_angle
            time.sleep(.5 * tag_angle)
        # if neato needs to move to the left
        elif tag_angle < 0:
            msg.angular.z = -.5 * tag_angle
            time.sleep(.5 * tag_angle)
            msg.linear.x = 0.5 * tag_angle
            time.sleep(.5 * tag_angle)

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