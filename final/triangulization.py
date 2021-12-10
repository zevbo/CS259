import random

from sympy.sets import EmptySet
from stuff import *
from sympy import symbols, solve, Eq, linsolve, core
import numpy as np
from specs import *


def get_solution_bad(t_sa, t_ab, meas_p_a, meas_p_b, z1, z2):

    meas_p_a2 = np.dot(t_ab, meas_p_b)
    zero_p = [a[0] for a in (meas_p_a2 - meas_p_a).tolist()[:-1]]

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

    return final_sol_sum / num_terms, np.dot(t_sa, meas_p_a)


def value_of(exp, v1, v2):
    return exp.evalf(subs={v1: 1.0, v2: 0.0}) - exp.evalf(subs={v1: 0.0, v2: 0.0})


def get_solution(t_sa, t_ab, meas_p_a, meas_p_b, z1, z2):

    meas_p_a2 = np.dot(t_ab, meas_p_b)
    zero_p = [a[0] for a in (meas_p_a2 - meas_p_a).tolist()[:-1]]

    def row(zero):
        return [value_of(zero, z1, z2), value_of(zero, z2, z1)]

    def constant_of(exp):
        return [- exp.evalf(subs={z1: 0.0, z2: 0.0})]

    m = np.array(list(map(row, zero_p)), dtype=float)
    print(m)
    y = np.array(list(map(constant_of, zero_p)))

    print("inverting: ")
    print(np.dot(m, np.transpose(m)))
    print("##")
    print(str(m))
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
    # sol, p = get_solution(t_sa, t_ab, meas_p_a, meas_p_b, z1, z2)
    sol, p = get_solution_bad(t_sa, t_ab, meas_p_a, meas_p_b, z1, z2)
    real_p = list(map(lambda el: 1 if el ==
                  1 else el[0].evalf(subs={z1: sol[0]}), p))
    real_p[2] += gripper_length
    return real_p
