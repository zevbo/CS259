from stuff import trap_arr
import numpy as np
import math

w1 = 133
w2 = 99.6
l1 = 425
l2 = 392
h1 = -240
h2 = 99.7

b_1 = trap_arr([0, 1, 0, w1 + w2, 0, l1 + l2])
b_2 = trap_arr([0, 0, 1, h2, -(l1 + l2), 0])
b_3 = trap_arr([0, 0, 1, h2, -l2, 0])
b_4 = trap_arr([0, 0, 1, h2, 0, 0])
b_5 = trap_arr([0, -1, 0, -w2, 0, 0])
b_6 = trap_arr([0, 0, 1, 0, 0, 0])

body_screws = [b_1, b_2, b_3, b_4, b_5, b_6]

link_lengths = [w1, l1, w1 + l2, w1 + w2, h2, 0]

def angle_cost(delta):
    weights = []
    curr_length = 0
    for length in link_lengths:
        curr_length += length
        weights.append(curr_length)
    return np.dot(abs(delta), trap_arr(list(reversed(weights))))

def legal_angle(angle):
    return angle[1] > math.pi / 20
    
joint_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']