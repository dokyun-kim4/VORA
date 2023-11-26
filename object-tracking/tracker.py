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
sort = Sort()
classified_objects = [39, 41, 64, 67, 73, 76]

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

    conf_box = h.get_obj_from_df(result_df,"cup")
    
    if len(conf_box) != 0:
        tracks = sort.update(conf_box)
    else:
        tracks = sort.update()
    
    for i in range(len(tracks)):
        x1,y1,x2,y2,id = tracks[i].astype(int)
        cv2.rectangle(frame,(x1,y1),(x2,y2),(0,0,255),2)
        cv2.putText(frame,str(id),(x1+10,y1+40),cv2.FONT_HERSHEY_PLAIN,3,(0,0,255),2)

    cv2.imshow("YOLO",frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows
