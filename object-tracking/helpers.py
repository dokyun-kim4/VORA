import cv2
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import torch
import numpy as np




def extract_confidence_bbox(results):
    """
    A helper function for extracting confidence and bounding box information
    from detected objects.
    """
    for result in results:
        bounding_box = result.boxes
        box_xyxy = bounding_box.xyxy.cpu().numpy()
        box_conf = bounding_box.conf.cpu().numpy()
    return box_xyxy, box_conf