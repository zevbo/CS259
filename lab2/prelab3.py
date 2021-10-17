from prelab2 import *
import numpy as np
from stuff import *


def trap_arr(arr):
    return np.transpose(np.array(arr))


b_1 = trap_arr([0, 1, 0, w1 + w2, 0, l1 + l2])
b_2 = trap_arr([0, 0, 1, h2, -(l1 + l2), 0])
b_3 = trap_arr([0, 0, 1, h2, -l2, 0])
b_4 = trap_arr([0, 0, 1, h2, 0, 0])
b_5 = trap_arr([0, -1, 0, -w2, 0, 0])
b_6 = trap_arr([0, 0, 1, 0, 0, 0])
bs = [b_1, b_2, b_3, b_4, b_5, b_6]


def calc_tsb_exp(thetas):
    m = np.array([
        [0, -1, 0, -(l1 + l2)],
        [0, 0, -1, -(w1 + w2)],
        [1, 0, 0, h1 - h2],
        [0, 0, 0, 1]
    ])
    exps = [se3_exp(b, theta) for (b, theta) in zip(bs, thetas)]
    return matmul(m, *exps)


def calc_j(thetas):

    curr_t = id(4)
    jacobian_tp = []
    for b, theta in zip(reversed(bs), reversed(thetas)):
        curr_transform = t_adjoint(curr_t)
        print("b")
        print(curr_transform)
        print(b)
        jacobian_tp.append(np.dot(curr_transform, b))
        curr_t = np.dot(curr_t, se3_exp(b, -1 * theta))

    return trap_arr(list(reversed(jacobian_tp)))


def get_vb(thetas, t_goal, e_w=0.01, e_v=1):
    result = np.dot(np.linalg.inv(calc_tsb_exp(thetas)), t_goal)
    screw, theta = se3_log(result)
    e_twist = screw * theta
    w = e_twist[0:3]
    v = e_twist[3:]
    print(np.linalg.norm(v))
    assert(np.linalg.norm(v) < 50)
    if np.linalg.norm(w) < e_w and np.linalg.norm(v) < e_v:
        return thetas
    else:
        jacobian = calc_j(thetas)
        new_thetas = thetas + np.dot(pseduo_inv(jacobian), e_twist)
        return get_vb(new_thetas, t_goal, e_w, e_v)


def test():
    theta1 = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    theta2 = [0.21, 0.31, 0.41, 0.51, 0.61, 0.71]
    t_goal = calc_tsb_exp(theta1)
    return get_vb(theta2, t_goal, 0.01, 0.01)
