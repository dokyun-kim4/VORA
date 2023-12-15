import cv2
import apriltag
import math
import time
from .april_tag_helpers import normalize_coordinates, draw_apriltag, calculate_april_angle, draw_april_angle, create_coord_pair
from geometry_msgs.msg import Twist

def locate_apriltag(apriltag_goal, apriltag_begin, apriltag_located, image, tag_family="tag36h11"):
        msg = Twist()
        if apriltag_goal is True: #If tag has already been reached, skip
            msg.linear.x = 0.0
            return msg, image, apriltag_goal, apriltag_begin, apriltag_located
        if apriltag_begin is False: #If tag detection has not begun yet, skip
            return msg, image, apriltag_goal, apriltag_begin, apriltag_located
        options = apriltag.DetectorOptions(families=tag_family)
        detector = apriltag.Detector(options)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        results = detector.detect(image)
        if apriltag_located is False: #while tag has not been located
            print("Looking for tag")
            if len(results) == 0: #if no tag, turn in a circle until tag detected
                msg.angular.z = -0.4
                msg.linear.x = 0.0
            else: #if there is a tag, turn until center of frame. # type: ignore
                coords = abs(create_coord_pair(image, results)[0])
                if abs(coords) < 50:
                    print("tag_located")
                    msg.angular.z = 0.0
                    msg.linear.x = 0.0
                    apriltag_located = True
                else:
                    msg.angular.z = -0.2
        if apriltag_located is True:
            image, tag_angle = draw_april_angle(image, results)
            tag_angle -= 90
            print(distance_check(results))
            distance = distance_check(results)
            #if abs(tag_angle) > 5:
                #msg = correct_for_angle(tag_angle, msg) #TODO: Create this function
                #msg.linear.x = 0.0
                #msg.angular.z = 0.0
                #apriltag_located = False
                #image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                #return msg, image, apriltag_goal, apriltag_begin, apriltag_located
            if distance < 60:
                msg.linear.x = 1.0
            else:
                apriltag_goal = True
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return msg, image, apriltag_goal, apriltag_begin, apriltag_located

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

def distance_check(results):
    corner = results[0][7]
    x_dist = corner[0][0] - corner[1][0]
    return x_dist