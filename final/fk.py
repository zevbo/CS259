
from stuff import *
from specs import *

def calc_tsb(thetas):
    exps = [se3_exp(b, theta) for (b, theta) in zip(body_screws, thetas)]
    return np.dot(m, matmul(*exps))