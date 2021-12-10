import math
from urllib.request import urlopen, urlretrieve
import cv2
import numpy as np
from stuff import *

url = "http://192.168.1.2:4242/current.jpg?annotations=off"
url_setup = "http://192.168.1.2:4242/captureimage"
filename = "image.jpg"


def getImage():
    urlopen(url_setup)
    urlretrieve(url, filename)
    return cv2.imread(filename)


maxHorAngle = 50 / 2 * math.pi / 180
maxVerAngle = 39 / 2 * math.pi / 180
maxX = 320
maxY = 240

# this is a guess rn
angle = 30 * math.pi / 180
r_bc = y_rotation(angle)
p_bc = np.array([42, 0, 19])
t_bc = r_and_shift_to_t(r_bc, p_bc)
