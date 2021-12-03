import random

from sympy.sets import EmptySet
from stuff import *
from sympy import symbols, solve, Eq, linsolve, core
import numpy as np


def get_solution(t_ab, meas_p_a, meas_p_b, z1, z2):

    meas_p_a2 = np.dot(t_ab, meas_p_b)
    zero_p = (meas_p_a2 - meas_p_a).tolist()

    sol = linsolve(zero_p, (z1, z2))

    if sol != EmptySet:
        return list(sol)[0]

    sol1 = linsolve([zero_p[0], zero_p[1]], (z1, z2))
    sol2 = linsolve([zero_p[0], zero_p[2]], (z1, z2))
    sol3 = linsolve([zero_p[1], zero_p[2]], (z1, z2))
    solutions = [sol1, sol2, sol3]
    final_sol_sum = np.array([0.0, 0.0])
    num_terms = 0

    def is_float(n):
        return isinstance(n, core.numbers.Float)

    for sol in solutions:

        if len(sol) == 1:
            spec_sol = np.array(list(sol)[0])
            if is_float(spec_sol[0]) and is_float(spec_sol[1]):
                spec_sol = np.array(list(map(float, spec_sol)))
                final_sol_sum += spec_sol
                num_terms += 1

    if num_terms == 0:
        raise RuntimeError("get_solution found no solution!")

    return final_sol_sum / num_terms

# returns real p_a


def triangulate(t_ab, meas_p_a, meas_p_b, z1, z2):
    meas_p_a = meas_p_a * z1
    meas_p_b = meas_p_b * z2
    sol = get_solution(t_ab, meas_p_a, meas_p_b, z1, z2)
    real_p = list(map(lambda el: 1 if el ==
                  1 else el.evalf(subs={z1: sol[0]}), meas_p_a))
    return real_p
