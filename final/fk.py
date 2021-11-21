
from stuff import *

w1 = 133
w2 = 99.6
l1 = 425
l2 = 392
h1 = -240
h2 = 99.7

## gotta dc this
b_1 = trap_arr([0, 1, 0, w1 + w2, 0, l1 + l2])
b_2 = trap_arr([0, 0, 1, h2, -(l1 + l2), 0])
b_3 = trap_arr([0, 0, 1, h2, -l2, 0])
b_4 = trap_arr([0, 0, 1, h2, 0, 0])
b_5 = trap_arr([0, -1, 0, -w2, 0, 0])
b_6 = trap_arr([0, 0, 1, 0, 0, 0])
bs = [b_1, b_2, b_3, b_4, b_5, b_6]

def calc_tsb(thetas):
    m = np.array([
        [1, 0, 0, -(l1 + l2)],
        [0, 0, -1, -(w1 + w2)],
        [0, 1, 0, h1 - h2],
        [0, 0, 0, 1]
    ])
    # pretty sure my thing is off by pi
    thetas[5] += math.pi
    exps = [se3_exp(b, 0 - theta) for (b, theta) in zip(bs, thetas)]
    return np.dot(m, matmul(*exps))