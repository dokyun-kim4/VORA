from geometry_msgs.msg import Twist
from ultralytics import YOLO
import cv2 as cv
from .helpers import get_tracks, find_obj, object_sortkey


def retrieve(image, obj_name):
    """
    Determines appropriate linear & angular velocity to go towards specified object
    """
    model = YOLO("yolov8n.pt")
    classified_objects = [39, 41, 64, 67, 73, 76]
    msg = Twist()
    results = model.predict(image, classes=classified_objects, verbose=False)
    track_all = get_tracks(results)
    for key in track_all:
        crnt_track = track_all[key]
        for i in range(len(crnt_track)):
            x1,y1,x2,y2,id = crnt_track[i].astype(int)
            cv.rectangle(image,(x1,y1),(x2,y2),(0,0,255),2) # type: ignore
            cv.putText(image,f"{key} {str(id)}",(x1+10,y1+40),cv.FONT_HERSHEY_PLAIN,2,(0,0,255),2) # type: ignore

    xy = find_obj(obj_name,track_all)
    if xy:

        if stop(obj_name,track_all):
            return None, image

        cv.circle(image,xy,5,(255,0,0),-1) # type: ignore
        x_norm = xy[0]/767 - 0.5
        thresh = 0.10          
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

    return msg, image

def stop(name,tracks):
    bbox = tracks[name][0]
    print(bbox[3])
    y_bottom_corner = bbox[3]
    if y_bottom_corner > 428:
        return True
    else:
        return False
