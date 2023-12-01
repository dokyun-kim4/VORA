import cv2
import apriltag
import argparse

# Goal - setup functions that will allow the robot to center itself between two april tags

cap = cv2.VideoCapture(0)


def normalize_coordinates(image, coordinates):
    height, width = image.shape[:2]
    image_center = [int(width/2), int(height/2)]
    # Subtract the center of frame from the input coordinates, making center of frame read [0, 0]
    coordinates[0] -= image_center[0]
    coordinates[1] -= image_center[1]
    return coordinates

def create_coord_pair(results):
    coordinate_list = []
    if len(results) < 2:
        return print("Not enough April Tags!")
    for coord in results:
        coordinates = coord[6]
        coordinate_list.append(coordinates)
    coordinate_list = coordinate_list[0:1]
    for coord in coordinate_list:
        coord = normalize_coordinates(image, coord)
    return coordinate_list

def movement_command(coord_pair, threshold):
    coord_sum = coord_pair[0][0]+coord_pair[1][0]
    if coord_sum < threshold and coord_sum > -threshold:
        print("You are now centered")
        return
    elif coord_sum > threshold:
        print("Turn left!")
        return
    elif coord_sum < -threshold:
        print("Turn right!")
        return


while True:
    ret, image = cap.read()
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(image)
    # If an april tag is detected, print out its center coordinates
    if not results:
        cv2.imshow("output",image)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        continue
    coordinate_list = create_coord_pair(results)
    if coordinate_list:
        print(coordinate_list)

    cv2.imshow("output",image)
    if cv2.waitKey(1) & 0xFF == ord("q"):
            break