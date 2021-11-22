import numpy as np
from stuff import *
import random

w1 = 133
w2 = 99.6
l1 = 425
l2 = 392
h1 = -240
h2 = 99.7

s1 = [0, 0, 1, 0, 0, 0]
s2 = [0, -1, 0, h1, 0, 0]
s3 = [0, -1, 0, h1, 0, l1]
s4 = [0, -1, 0, h1, 0, l1+l2]
s5 = [0, 0, -1, w1, -(l1+l2), 0]
s6 = [0, -1, 0, h1-h2, 0, l1+l2]
screws = [s1, s2, s3, s4, s5, s6]

def calc_tsb_exp(thetas):
    m = np.array([
        [0, -1, 0, -(l1 + l2)],
        [0, 0, -1, -(w1 + w2)],
        [1, 0, 0, h1 - h2],
        [0, 0, 0, 1]
    ])
    exps = [se3_exp(b, theta) for (b, theta) in zip(screws, thetas)]
    return np.dot(matmul(*exps), m)


def get_thetas(thetas, t_goal, max_iters=100, e_w=0.01, e_v=1):
    while(max_iters > 0):
        max_iters -= 1
        result = np.dot(np.linalg.inv(calc_tsb_exp(thetas)), t_goal)
        screw, theta = se3_log(result)
        e_twist = screw * theta
        w = e_twist[0:3]
        v = e_twist[3:]
        if np.linalg.norm(w) < e_w and np.linalg.norm(v) < e_v:
            return thetas
        else:
            jacobian = calc_space_j(thetas, screws)
            thetas += np.dot(pseduo_inv(jacobian), e_twist)
    return None

def get_thetas_persistent(t_goal, thetas=None, persistent_tries = 20):
    while(persistent_tries > 0):
        persistent_tries -= 1
        if thetas == None:
            thetas = np.array(list(map(lambda i: random.random() * 2 * math.pi, range(6))))
        val = get_thetas(thetas, t_goal)
        if val != None:
            return val
        thetas = None
