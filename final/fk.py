
from stuff import *
from specs import *

def calc_tsb(thetas):
    m = np.array([
        [0, -1, 0, -(l1 + l2)],
        [0, 0, -1, -(w1 + w2)],
        [1, 0, 0, h1 - h2],
        [0, 0, 0, 1]
    ])
    exps = [se3_exp(b, theta) for (b, theta) in zip(body_screws, thetas)]
    return np.dot(m, matmul(*exps))