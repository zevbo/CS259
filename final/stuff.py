import numpy as np
from numpy.linalg import matrix_power
import math


def trap_arr(arr):
    return np.transpose(np.array(arr))


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

def r_and_shift_to_t(r, shift):
    return np.dot(shift_to_t(shift), r_to_t(r))

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
        [0, -w[2], w[1]],
        [w[2], 0, -w[0]],
        [-w[1], w[0], 0]
    ])


def so3_log(r):
    theta = math.acos((r[0][0] + r[1][1] + r[2][2] - 1) / 2)
    if (math.sin(theta) == 0):
        return np.array([0, 0, 0]), theta

    def get_w(i1, i2):
        return (r[i1][i2] - r[i2][i1]) / (2 * math.sin(theta))
    w1 = get_w(2, 1)
    w2 = get_w(0, 2)
    w3 = get_w(1, 0)
    return (np.array([w1, w2, w3]), theta)


def id(n):
    arr = np.array([[0] * n for _ in range(n)])
    for i in range(n):
        arr[i][i] = 1
    return arr


def se3_log(t):
    r = t[0:3, 0:3]
    p = t[0:3, 3]
    w, theta = so3_log(r)
    if theta == 0:
        return np.array([0, 0, 0, 0, 0, 0]), theta
    w_m = lp_cross(w)
    i = id(3)
    g_inv = (1 / theta) * i - (1 / 2) * w_m + (1 / theta -
                                               (1 / 2) / math.tan(theta / 2)) * np.dot(w_m, w_m)
    v = np.transpose(np.dot(g_inv, p))
    screw = np.concatenate((w, v), axis=0)
    return screw, theta


def so3_exp(w, theta):
    i = id(3)
    w_m = lp_cross(w)
    return i + math.sin(theta) * w_m + (1 - math.cos(theta)) * np.dot(w_m, w_m)


## Vertified: Se3_log(se3_exp(s, t)) = s, t
def se3_exp(screw, theta):
    w = screw[0:3]
    v = screw[3:]
    top_left = so3_exp(w, theta)
    i = id(3)
    w_m = lp_cross(w)
    top_right = np.dot(i * theta + (1 - math.cos(theta)) *
                       w_m + (theta - math.sin(theta)) *
                       np.dot(w_m, w_m), np.transpose(np.array([v])))
    top = np.concatenate((top_left, top_right), axis=1)
    bot = np.array([[0, 0, 0, 1]])
    return np.concatenate((top, bot), axis=0)


def t_adjoint(t):
    r = t[0:3, 0:3]
    p = t[0:3, 3]
    p_m = lp_cross(p)
    bot_left = np.dot(p_m, r)
    top_right = np.array([[0] * 3 for _ in range(3)])
    top = np.concatenate((r, top_right), axis=1)
    bot = np.concatenate((bot_left, r), axis=1)
    adj = np.concatenate((top, bot), axis=0)
    return adj


def pseduo_inv(m):
    m_tp = np.transpose(m)
    return np.dot(m_tp, np.linalg.inv(np.dot(m, m_tp)))


def calc_body_j(thetas, bs):

    curr_t = id(4)
    jacobian_tp = []
    for b, theta in zip(reversed(bs), reversed(thetas)):
        curr_transform = t_adjoint(curr_t)
        jacobian_tp.append(np.dot(curr_transform, b))
        curr_t = np.dot(curr_t, se3_exp(b, -1 * theta))

    return trap_arr(list(reversed(jacobian_tp)))


def calc_space_j(thetas, screws):

    curr_t = id(4)
    jacobian_tp = []
    for b, theta in zip(screws, thetas):
        curr_transform = t_adjoint(curr_t)
        jacobian_tp.append(np.dot(curr_transform, b))
        curr_t = np.dot(curr_t, se3_exp(b, theta))

    return trap_arr(jacobian_tp)


