import cv2
import apriltag
import math
import time
from .april_tag_helpers import normalize_coordinates, draw_apriltag, calculate_april_angle, draw_april_angle, create_coord_pair
from geometry_msgs.msg import Twist

def locate_apriltag(apriltag_goal, apriltag_begin, apriltag_located, frame, tag_family="tag36h11"):
        msg = Twist()
        if apriltag_goal is True: #If tag has already been reached, skip
            return
        if apriltag_begin is False: #If tag detection has not begun yet, skip
            return
        options = apriltag.DetectorOptions(families=tag_family)
        detector = apriltag.Detector(options)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        print(len(frame.shape))
        results = detector.detect(frame)
        if apriltag_located is False: #while tag has not been located
            if len(results) == 0: #if no tag, turn in a circle until tag detected
                msg.angular.z = -0.2
                msg.linear.x = 0.0
            elif abs(create_coord_pair(results)[0]) < 50: #if there is a tag, turn until center of frame. # type: ignore
                msg.angular.z = 0.0
                msg.linear.x = 0.0
                apriltag_located = True
        if apriltag_located is True:
            frame, tag_angle = draw_april_angle(frame, results)
            if abs(tag_angle) > 5:
                msg = correct_for_angle(tag_angle, msg) #TODO: Create this function
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                apriltag_located = False
                return
            msg.linear.x = 2.0
            apriltag_goal = True
        return msg

def correct_for_angle(tag_angle, msg):
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
        return msg