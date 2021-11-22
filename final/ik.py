import numpy as np
from stuff import *
import random
from specs import *
from fk import calc_tsb


def get_thetas(thetas, t_goal, e_w=0.01, e_v=1, max_iters=100):
    while(max_iters > 0):
        max_iters -= 1
        result = np.dot(np.linalg.inv(calc_tsb(thetas)), t_goal)
        screw, theta = se3_log(result)
        e_twist = screw * theta
        w = e_twist[0:3]
        v = e_twist[3:]
        if np.linalg.norm(w) < e_w and np.linalg.norm(v) < e_v:
            return thetas
        else:
            jacobian = calc_body_j(thetas, body_screws)
            thetas += np.dot(pseduo_inv(jacobian), e_twist)
    return None

def get_thetas_persistent(t_goal, thetas_ideal, persistent_tries = 20):
    best_result = None
    best_error = 0
    thetas = thetas_ideal
    while(persistent_tries > 0):
        persistent_tries -= 1
        if thetas is None:
            thetas = np.array(list(map(lambda i: random.random() * 2 * math.pi, range(6))))
        result = get_thetas(thetas, t_goal)
        if not(result is None):
            error = np.linalg.norm(result - thetas_ideal)
            if best_result is None or error < best_error:
                best_error = result 
                best_error = error
        thetas = None
    return best_result


def tester(theta, delta):
    theta_guess = theta + delta
    t_goal = calc_tsb(theta)
    return get_thetas_persistent(t_goal, theta_guess)