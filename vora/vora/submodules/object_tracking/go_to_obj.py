from geometry_msgs.msg import Twist
from ultralytics import YOLO
import cv2 as cv
from .helpers import get_tracks, find_obj, object_sortkey
import numpy as np
from typing import Union


def retrieve(image: np.ndarray, obj_name: str)->Union[Twist, None, np.ndarray]:
    """
    Determines appropriate linear & angular velocity to go towards specified object

    Args:
        image (np.ndarray): current video frame
        obj_name (str): object to look for in the frame
    
    Returns:
        msg (Twist): linear and angular velocity the Neato should have
        image (np.ndarray): current video frame with object identified with bounding boxes
    """
    model = YOLO("yolov8n.pt")
    classified_objects = [39, 41, 64, 67, 73, 76]
    msg = Twist()

    image = enhance(image)

    # Identify all objects in `classified_objects`
    results = model.predict(image, classes=classified_objects, verbose=False)
    track_all = get_tracks(results)
    for key in track_all:
        crnt_track = track_all[key]
        for i in range(len(crnt_track)):
            x1,y1,x2,y2,id = crnt_track[i].astype(int)
            # Draw bounding boxes
            cv.rectangle(image,(x1,y1),(x2,y2),(0,0,255),2) # type: ignore
            cv.putText(image,f"{key} {str(id)}",(x1+10,y1+40),cv.FONT_HERSHEY_PLAIN,2,(0,0,255),2) # type: ignore

    # Find center coordinates of specified object
    xy = find_obj(obj_name,track_all)
    if xy:
        # Check if object has been obtained
        if stop(obj_name,track_all):
            return None, image #type: ignore

        cv.circle(image,xy,5,(255,0,0),-1) # type: ignore
        x_norm = xy[0]/767 - 0.5
        thresh = 0.10 

        # Return appropriate velocity depending on location of the center point         
        if x_norm >= -thresh and x_norm <= thresh:
            msg.linear.x = 0.05
        elif x_norm < -thresh:
            msg.linear.x = 0.0
            msg.angular.z = 0.2
        elif x_norm > thresh:
            msg.linear.x = 0.0
            msg.angular.z = -0.2
    else:
        msg.linear.x = 0.0
        msg.angular.x = 0.0
    return msg, image # type: ignore

def stop(name: str,tracks: dict)->bool:
    """
    Check if object has been obtained

    Args:
        name (str): Name of object being tracked
        tracks (dict): Dictionary containing tracks of all objects

    Returns:
        stop (bool): boolean indicating if the object has been obtained
    """
    stop = False
    bbox = tracks[name][0]
    y_bottom_corner = bbox[3]
    if y_bottom_corner > 430:
        stop = True
        return stop
    else:
        return stop


def enhance(image: np.ndarray)->np.ndarray:
    """
    Enhances the contrast of a given image

    Args:
        image (np.ndarray): image to enhance
    
    Returns:
        enhanced_img (np.ndarray): enhanced image
    """
    lab= cv.cvtColor(image, cv.COLOR_BGR2LAB)
    l_channel, a, b = cv.split(lab)
    clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    cl = clahe.apply(l_channel)

    limg = cv.merge((cl,a,b))
    enhanced_img = cv.cvtColor(limg, cv.COLOR_LAB2BGR)

    return enhanced_img