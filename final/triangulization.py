import random

from sympy.sets import EmptySet
from stuff import *
from sympy import symbols, solve, Eq, linsolve, core
import numpy as np
from specs import *


def value_of(exp, v1, v2):
    return exp.evalf(subs={v1: 1.0, v2: 0.0}) - exp.evalf(subs={v1: 0.0, v2: 0.0})


def row(zero, z1, z2):
    return [value_of(zero, z1, z2), value_of(zero, z2, z1)]


def constant_of(exp, v1, v2):
    return - exp.evalf(subs={v1: 0.0, v2: 0.0})


def get_solution(t_sa, t_ab, meas_p_a, meas_p_b, z1, z2):

    meas_p_a2 = np.dot(t_ab, meas_p_b)
    zero_p = [a[0] for a in (meas_p_a2 - meas_p_a).tolist()[:-1]]

    m = np.array([row(zero_p[0], z1, z2), row(
        zero_p[1], z1, z2), row(zero_p[2], z1, z2)])
    y = np.transpose(np.array(
        [[constant_of(zero_p[0]), constant_of(zero_p[1]), constant_of(zero_p[2])]]))

    x = np.dot(pseduo_inv(m), y)

    return x[0], np.dot(t_sa, meas_p_a)

# returns real p_a


def get_solution_simple(t_sc, meas_p, z, height):
    p = np.dot(t_sc, meas_p)
    return list(linsolve([p[2][0] - height], (z)))[0], p


def triangulate(t_ab, meas_p_a, z1, height):
    sol, p = get_solution_simple(t_ab, meas_p_a, z1, height)
    real_p = list(map(lambda el: 1 if el ==
                  1 else el[0].evalf(subs={z1: sol[0]}), p))
    real_p[2] += gripper_length
    return real_p


def triangulate2(t_sa, t_ab, meas_p_a, meas_p_b, z1, z2):
    #sol = get_solution(t_ab, meas_p_a, meas_p_b, z1, z2)
    sol, p = get_solution(t_sa, t_ab, meas_p_a, meas_p_b, z1, z2)
    real_p = list(map(lambda el: 1 if el ==
                  1 else el[0].evalf(subs={z1: sol[0]}), p))
    real_p[2] += gripper_length
    return real_p
