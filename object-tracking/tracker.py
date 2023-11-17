#TODO: setup YOLO to only track and classify objects in our list
import cv2
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import torch
import numpy as np


model = YOLO("yolov8m.pt")
cap = cv2.VideoCapture(0)
classified_objects = [39, 41, 64, 67, 73, 76]

while cap.isOpened():
    success, frame = cap.read()
    results = model.predict(frame, classes=classified_objects, verbose=False)

    for r in results:
        annotator = Annotator(frame)

        bboxes = r.boxes
        box_count = 0
        for box in bboxes:
            box_count += 1
            box_coord = box.xyxy[0]
            boxclass = box.cls
            annotator.box_label(box_coord, f"{model.names[int(boxclass)]} {box_count}")
    
    annotated_frame = annotator.result()
    cv2.imshow("YOLO", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows
