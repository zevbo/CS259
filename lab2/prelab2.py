import numpy as np
from numpy.lib.index_tricks import r_
from stuff import *
import math

w1 = 133
w2 = 99.6
l1 = 425
l2 = 392
h1 = -240
h2 = 99.7


def calc_tsb(t1, t2, t3, t4, t5, t6):

    def dh_matrix(x_rot, x_shift, z_shift, z_rot):
        x_rot_t = r_to_t(x_rotation(x_rot))
        x_shift_t = shift_to_t([x_shift, 0, 0])
        z_shift_t = shift_to_t([0, 0, z_shift])
        z_rot_t = r_to_t(z_rotation(z_rot))
        return matmul(z_rot_t, z_shift_t, x_shift_t, x_rot_t)

    # not sure if it is positive or negative h1
    # not sure if this offset is supposed to be w1
    # Note, I am adding an extra math.pi to the original rotation, and acting is if the positive x and y directions are the same as in the picture
    t01 = dh_matrix(-math.pi/2, 0, h1, t1 + math.pi)
    t12 = dh_matrix(0, l1, 0, t2)
    t23 = dh_matrix(0, l2, 0, t3)
    t34 = dh_matrix(-math.pi / 2, 0, w1, t4)
    t45 = dh_matrix(math.pi / 2, 0, h2, t5)
    t56 = dh_matrix(0, 0, w2, t6 - math.pi / 2)
    #print("transformations: ")
    return matmul(t01, t12, t23, t34, t45, t56)


def to_rad(deg):
    return deg * math.pi / 180


def eof_pos(t1, t2, t3, t4, t5, t6):
    tsb = calc_tsb(to_rad(t1), to_rad(t2), to_rad(t3),
                   to_rad(t4), to_rad(t5), to_rad(t6))
    m = np.array([[0.0], [0.0], [0.0], [1.0]])
    eof_pos = matmul(tsb, m)
    x = eof_pos[0]
    y = eof_pos[1]
    z = eof_pos[2]
    if (x >= 0 or y <= -500 or y >= 500 or z <= -400):
        print("WARNING, OUT OF RANGE CONFIGURATION")
    return eof_pos
