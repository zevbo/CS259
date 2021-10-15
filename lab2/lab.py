from prelab import *
import matplotlib.pyplot as plt
import time


def get_points(prc):
    pts = []
    for t2 in range(-45, 0, prc):
        for t3 in range(-90, -30, prc):
            for t4 in range(-45, 45, prc):
                pts.append(eof_pos(0, t2, t3, t4, 0, 0))
    return pts


def make_graph(prc):
    pts = get_points(prc)
    xs = np.array(list(map(lambda pt: pt[0], pts)))
    zs = np.array(list(map(lambda pt: pt[2], pts)))
    plt.style.use('seaborn-whitegrid')
    plt.plot(xs, zs, 'o', color='black')
    plt.show()
