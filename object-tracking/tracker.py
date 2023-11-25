#TODO: setup YOLO to only track and classify objects in our list
import cv2
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import torch
import numpy as np
import helpers as h

model = YOLO("yolov8m.pt")
cap = cv2.VideoCapture(0)

classified_objects = [39, 41, 64, 67, 73, 76]


while cap.isOpened():
    success, frame = cap.read()
    annotator = Annotator(frame)
    results = model.predict(frame, classes=classified_objects, verbose=False)
    # print(results)
    list, conf, name = h.extract_confidence_bbox(results)
    print(conf)
    for r in results:
        
        bboxes = r.boxes
        bound_box = bboxes.xyxy
        prob = bboxes.conf.cpu().numpy()
        for box in bboxes:

            box_coord = box.xyxy[0]
            boxclass = box.cls
            annotator.box_label(box_coord, f"{h.objects[int(boxclass)]}")

    
    annotated_frame = annotator.result()
    cv2.imshow("YOLO", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows
