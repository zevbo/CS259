import numpy as np
from numpy.linalg import matrix_power
import math


def z_rotation(angle):
    cos = math.cos(angle)
    sin = math.sin(angle)
    return np.array(
        [[cos, -sin, 0],
         [sin, cos, 0],
         [0, 0, 1]])


def y_rotation(angle):
    cos = math.cos(angle)
    sin = math.sin(angle)
    return np.array(
        [[cos, 0, sin],
         [0, 1, 0],
         [-sin, 0, cos]])


def x_rotation(angle):
    cos = math.cos(angle)
    sin = math.sin(angle)
    return np.array(
        [[1, 0, 0],
         [0, cos, -sin],
         [0, sin, cos]])


def print_matrix(mat):
    for row in mat:
        for val in row:
            print(val, end="")
            print("\n", end="")
        print("")
