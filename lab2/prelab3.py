from lab2.prelab2 import calc_tsb
import prelab2 as pl2
import numpy as np
from stuff import *


def calc_j(thetas):
    b_1 = np.transpose(np.array([0, 1, 0, 0, 191, 0, 817]))
    b_2 = np.transpose(np.array([0, 0, 1, 95, -817, 0]))
    b_3 = np.transpose(np.array([0, 0, 1, 95, -392, 0]))
    b_4 = np.transpose(np.array([0, 0, 1, 95, 0, 0]))
    b_5 = np.transpose(np.array([0, -1, 0, -82, 0, 0]))
    b_6 = np.transpose(np.array([0, 0, 1, 0, 0, 0]))
    return 0


def get_vb(thetas, t_goal, e_w, e_v):
    result = np.dot(np.linalg.inv(calc_tsb(*thetas)), t_goal)
    w, v = se3_log(result)
    if np.absolute(w) < e_w and np.absolute(v) < e_v:
        return w, v
    else:
        twist = np.array([[w[0]], [w[1]], [w[2]], [v[0]], [v[1]], [v[2]]])
        new_thetas = 0
