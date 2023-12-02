#TODO: setup YOLO to only track and classify objects in our list
import cv2
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import torch
import numpy as np
import helpers as h
from sort import Sort
import time

model = YOLO("yolov8m.pt")
cap = cv2.VideoCapture(0)

classified_objects = [39, 41, 64, 67, 73, 76]
classified_objects_names = h.object_sortkey

prev_frame = None
prev_box = None

for _ in range(10):
# Take a few frames so the camera can adjust
    cap.read()
time.sleep(0.5)

while cap.isOpened():

    success, frame = cap.read()
    results = model.predict(frame, classes=classified_objects, verbose=False)
    result_df  = h.convert_to_df(results)
    print(result_df)

    conf_box_all = {}
    for name in classified_objects_names:
        conf_box = h.get_obj_from_df(result_df,name)
        conf_box_all[name] = conf_box




    track_all = {}
    for i,name in enumerate(classified_objects_names):
        crnt_conf_box = conf_box_all[name]
        print(name)
        print(crnt_conf_box)
        if len(crnt_conf_box) != 0:
 
            track_all[name] = h.sorter[i].update(crnt_conf_box)

        else:

            track_all[name] = h.sorter[i].update()

    print(track_all)

    for key in track_all:
        crnt_track = track_all[key]
        for i in range(len(crnt_track)):
            x1,y1,x2,y2,id = crnt_track[i].astype(int)
            cv2.rectangle(frame,(x1,y1),(x2,y2),(0,0,255),2)
            cv2.putText(frame,f"{key} {str(id)}",(x1+10,y1+40),cv2.FONT_HERSHEY_PLAIN,3,(0,0,255),2)

    cv2.imshow("YOLO",frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows
