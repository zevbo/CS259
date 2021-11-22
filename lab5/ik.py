import numpy as np
from numpy.linalg import norm
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
            normalize_vec(thetas)
            return thetas
        else:
            jacobian = calc_body_j(thetas, body_screws)
            thetas += np.dot(pseduo_inv(jacobian), e_twist)
    return None

# weird behavior on:
t_goal = np.array(
[[   1.,    0.,    0., -200.],
 [   0.,    1.,    0.,  -50.],
 [   0.,    0.,    1.,    0.],
 [   0.,    0.,    0.,    1.]])
# and thetas = [-0.6411795,  -1.61797559,  2.42879098,  0.75998094, -1.57079633, -2.50041316]

def get_thetas_persistent(t_goal, thetas_ideal, persistent_tries = 20, e_w=0.01, e_v=1):
    best_result = None
    best_error = 0
    thetas = np.array(thetas_ideal, copy=True)
    while(persistent_tries > 0):
        persistent_tries -= 1
        result = get_thetas(thetas, t_goal, e_w=e_w, e_v=e_v)
        if not(result is None) and legal_angle(result):
            thetas
            diff = result - thetas_ideal 
            normalize_vec(diff)
            error = angle_cost(diff)
            if best_result is None or error < best_error:
                # print("previous best was " + str(best_result) + " with error of " + str(best_error) + ". Improved to " + str(result) + " with error of " + str(error))
                best_result = result 
                best_error = error
        thetas = np.array(list(map(lambda i: random.random() * 2 * math.pi, range(6))))
    return best_result


def tester(theta, delta):
    theta_guess = theta + delta
    t_goal = calc_tsb(theta)
    return get_thetas_persistent(t_goal, theta_guess)