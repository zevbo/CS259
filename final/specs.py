from stuff import *
import numpy as np
import math

w1 = 133
w2 = 99.6
l1 = 425
l2 = 392
h1 = -240
h2 = 99.7

s_1 = trap_arr([0, 0, 1, 0, 0, 0])
s_2 = trap_arr([0, -1, 0, h1, 0, 0])
s_3 = trap_arr([0, -1, 0, h1, 0, l1])
s_4 = trap_arr([0, -1, 0, h1, 0, l1 + l2])
s_5 = trap_arr([0, 0, -1, w1, -(l1 + l2), 0])
s_6 = trap_arr([0, -1, 0, h1 - h2, 0, l1 + l2])

m = np.array([
    [0, -1, 0, -(l1 + l2)],
    [0, 0, -1, -(w1 + w2)],
    [1, 0, 0, h1 - h2],
    [0, 0, 0, 1]
])

screws = [s_1, s_2, s_3, s_4, s_5, s_6]

body_screws = [s_to_b(s, m) for s in screws]

link_lengths = [w1, l1, w1 + l2, w1 + w2, h2, 0]

def angle_cost(delta):
    weights = []
    curr_length = 0
    for length in link_lengths:
        curr_length += length
        weights.append(curr_length)
    return np.dot(abs(delta), trap_arr(list(reversed(weights))))

def legal_angle(angle):
    return angle[1] <= math.pi / 40

joint_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
