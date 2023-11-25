#TODO: setup YOLO to only track and classify objects in our list
import cv2
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import torch
import numpy as np
import helpers as h
from sort import Sort

model = YOLO("yolov8m.pt")
cap = cv2.VideoCapture(0)
sort = Sort()
classified_objects = [39, 41, 64, 67, 73, 76]

prev_frame = None
prev_box = None

while cap.isOpened():
    success, frame = cap.read()
    results = model.predict(frame, classes=classified_objects, verbose=False)
    result_df  = h.convert_to_df(results)

    bbox, conf = h.get_obj_from_df(result_df,"cup")




    cv2.imshow("YOLO",frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows
