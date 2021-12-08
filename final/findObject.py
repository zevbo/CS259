import numpy as np
import cv2
from camera import *

# Todo: choose reasonable area
min_contour_area = 20


def result_to_image(result):
    str_result = str(result).split("[")[-1][0:-1].split(", ")
    arr = list(map(lambda s: int(s), str_result))
    data = []
    for row in range(result.height):
        data.append([])
        for col in range(result.width):
            PIXEL_SIZE = 3
            index = row * result.width * PIXEL_SIZE + col * PIXEL_SIZE
            data[-1].append(arr[index:index + PIXEL_SIZE])
    return np.array(data, dtype=np.dtype('uint8'))


def find_centroid(mask):
    contours = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    big_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(big_contour) < min_contour_area:
        return None
    print("Area: ", cv2.contourArea(big_contour))
    M = cv2.moments(big_contour)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx - maxX, cy - maxY), big_contour


lower_red = np.array([0, 100, 150])
upper_red = np.array([10, 255, 255])

lower_green = np.array([170, 50, 50])
upper_green = np.array([180, 255, 255])


def find_obj(img):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower mask (0-10)
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

    # upper mask (170-180)
    mask1 = cv2.inRange(img_hsv, lower_green, upper_green)

    return find_centroid(mask0 + mask1)


def find_dest(img):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_green = np.array([110, 50, 50])
    upper_green = np.array([130, 255, 255])
    mask = cv2.inRange(img_hsv, lower_green, upper_green)
    return find_centroid(mask)
