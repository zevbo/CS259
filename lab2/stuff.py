import numpy as np
from numpy.linalg import matrix_power
import math


def z_rotation(angle):
    cos = math.cos(angle)
    sin = math.sin(angle)
    return np.array(
        [[cos, -sin, 0],
         [sin, cos, 0],
         [0, 0, 1.0]])


def y_rotation(angle):
    cos = math.cos(angle)
    sin = math.sin(angle)
    return np.array(
        [[cos, 0, sin],
         [0, 1.0, 0],
         [-sin, 0, cos]])


def x_rotation(angle):
    cos = math.cos(angle)
    sin = math.sin(angle)
    return np.array(
        [[1.0, 0, 0],
         [0, cos, -sin],
         [0, sin, cos]])


def r_to_t(r):
    t = np.array([[0.0] * 4 for _ in range(4)])
    for i in range(3):
        for j in range(3):
            t[i][j] = r[i][j]
    t[3][3] = 1
    return t


def shift_to_t(shift):
    t = np.array([[0.0] * 4 for _ in range(4)])
    for i in range(len(shift)):
        t[i][3] = shift[i]
        t[i][i] = 1
    t[3][3] = 1
    return t


def matmul(*matricies):
    res = matricies[0]
    for m in matricies[1:]:
        res = np.matmul(res, m)
    return res


def print_matrix(mat):
    for row in mat:
        for val in row:
            print(val, end="")
            print("\n", end="")
        print("")


def lp_cross(w):
    return np.array([
        [0, w[2], -w[1]],
        [-w[2], 0, w[0]],
        [w[1], w[0], 0]
    ])


def so3_log(r):
    theta = math.acos((r[1][1] + r[2][2] + r[3][3] - 1) / 2)
    w1 = (r[2][1] - r[1][2]) / math.sin(theta)
    w2 = (r[0][2] - r[2][0]) / math.sin(theta)
    w3 = (r[1][0] - r[0][1]) / math.sin(theta)
    return (np.array([w1, w2, w3]), theta)


def id(n):
    arr = np.array([[0] * n for _ in range(n)])
    for i in range(n):
        arr[i][i] = 1
    return arr


def se3_log(t):
    r = t[0:3][0:3]
    p = np.array([[t[0][3]], [t[1][3]], [t[2][3]]])
    w, theta = so3_log(r)
    w_m = lp_cross(w)
    i = id(3)
    g_inv = (1 / theta) * i - (1 / 2) * w_m + (1 / theta -
                                               (1 / 2) / math.tan(theta / 2)) * np.dot(w_m, w_m)
    v = np.dot(g_inv, p)
    return w, v


def so3_exp(w, theta):
    i = id(3)
    w_m = lp_cross(w)
    return i + math.sin(theta) * w_m + (1 - math.cos(theta)) * np.dot(w_m, w_m)


def se3_exp(screw, theta):
    w = screw[0:3]
    v = screw[3:]
    top_left = so3_exp(w, theta)
    i = id(3)
    w_m = lp_cross(w)
    top_right = np.dot(i * theta + (1 - math.cos(theta)) *
                       w_m + (theta - math.sin(theta)) * np.dot(w_m, w_m))
    top = np.concatenate((top_left, top_right), axis=1)
    bot = np.array([[0, 0, 0, 1]])
    return np.concatenate((top, bot), axis=0)
