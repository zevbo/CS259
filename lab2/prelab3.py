from prelab2 import *
import numpy as np
from stuff import *


b_1 = trap_arr([0, 1, 0, w1 + w2, 0, l1 + l2])
b_2 = trap_arr([0, 0, 1, h2, -(l1 + l2), 0])
b_3 = trap_arr([0, 0, 1, h2, -l2, 0])
b_4 = trap_arr([0, 0, 1, h2, 0, 0])
b_5 = trap_arr([0, -1, 0, -w2, 0, 0])
b_6 = trap_arr([0, 0, 1, 0, 0, 0])
bs = [b_1, b_2, b_3, b_4, b_5, b_6]


def calc_tsb_exp(thetas):
    m = np.array([
        [1, 0, 0, -(l1 + l2)],
        [0, 0, -1, -(w1 + w2)],
        [0, 1, 0, h1 - h2],
        [0, 0, 0, 1]
    ])
    exps = [se3_exp(b, theta) for (b, theta) in zip(bs, thetas)]
    return np.dot(m, matmul(*exps))


def get_vb(thetas, t_goal, e_w=0.01, e_v=1):
    while(True):
        result = np.dot(np.linalg.inv(calc_tsb(*thetas)), t_goal)
        screw, theta = se3_log(result)
        e_twist = screw * theta
        w = e_twist[0:3]
        v = e_twist[3:]
        if np.linalg.norm(w) < e_w and np.linalg.norm(v) < e_v:
            return thetas
        else:
            jacobian = calc_body_j(thetas, bs)
            thetas += np.dot(pseduo_inv(jacobian), e_twist)


def normalize_angle(rad):
    reg = rad % (2 * 3.14159)
    deg = reg * 180 / 3.14159
    if (deg > 180):
        return deg - 360
    else:
        return deg


def get_vb_normalize(thetas, t_goal):
    real_thetas = get_vb(thetas, t_goal)
    return [round(normalize_angle(theta) * 100) / 100 for theta in real_thetas]


def test(theta, delta):
    theta_guess = theta + delta
    t_goal = calc_tsb(*theta)
    return get_vb(theta_guess, t_goal, 0.01, 0.01)


def test2():
    m = np.array([
        [1, 0, 0, -(l1 + l2)],
        [0, 0, -1, -(w1 + w2)],
        [0, 1, 0, h1 - h2],
        [0, 0, 0, 1]
    ])
    m_i = np.linalg.inv(m)
    thetas_v = [[1 if i == j else 0 for i in range(6)] for j in range(6)]
    for thetas in thetas_v:
        print(np.dot(m_i, calc_tsb(*thetas)))
    print("NOW BAD")
    for thetas in thetas_v:
        print(np.dot(m_i, calc_tsb_exp(thetas)))
