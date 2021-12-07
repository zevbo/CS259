import math
from urllib.request import urlretrieve
import cv2
import numpy as np
from stuff import *

url = "http://192.168.1.2:4242/current.jpg?annotations=off"
filename = "image.jpg"


def getImage():
    urlretrieve(url, filename)
    return cv2.imread(filename)


maxHorAngle = math.pi / 4
maxVerAngle = 0.341
maxX = 240
maxY = 320

# this is a guess rn
angle = 30 * math.pi / 180
r_bc = x_rotation(angle)
p_bc = np.array([0, -4.2, -15])
t_bc = r_and_shift_to_t(r_bc, p_bc)
