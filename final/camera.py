import math
from urllib.request import urlretrieve
import cv2

url = "http://192.168.1.2:4242/current.jpg?annotations=off"
filename = "image.jpg"


def getImage():
    urlretrieve(url, filename)
    return cv2.imread(filename)


maxHorAngle = math.pi / 4
maxVerAngle = math.pi / 6
maxX = 240
maxY = 320
