import cv2
import apriltag
import argparse

cap = cv2.VideoCapture(0)

while True:
    ret, image = cap.read()
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(image)
    print(results)
    cv2.imshow("output",image)
    if cv2.waitKey(1) & 0xFF == ord("q"):
            break