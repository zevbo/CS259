import random

from sympy.sets import EmptySet
from stuff import *
from sympy import symbols, solve, Eq, linsolve, core
import numpy as np

z1 = symbols('z1')
z2 = symbols('z2')

def fix_pos(pos):
    return np.append(pos, np.array([[1]]))

def to_measurment(real_p, sym):
    pos = real_p[:-1]
    mag = np.linalg.norm(pos)
    new_pos = pos / mag * sym 
    return fix_pos(new_pos)

theta_x = random.uniform(0.0, 2 * math.pi)
theta_y = random.uniform(0.0, 2 * math.pi)
theta_z = random.uniform(0.0, 2 * math.pi)

r_ab = matmul(z_rotation(theta_z), y_rotation(theta_y), x_rotation(theta_x))
p_ab = np.array([2, -3, 7])
t_ab = r_to_t(r_ab) + shift_to_t(p_ab)
t_ab[3][3] = 1
t_ba = np.linalg.inv(t_ab)

real_p_a = np.transpose(np.array([1, -2, 4, 1]))
real_p_b = np.dot(t_ba, real_p_a)

meas_p_a = to_measurment(real_p_a, z1)
meas_p_b = to_measurment(real_p_b, z2)

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
    sol = get_solution(t_ab, meas_p_a, meas_p_b, z1, z2)
    print(meas_p_a)
    real_p = list(map(lambda el: 1 if el == 1 else el.evalf(subs={z1: sol[0]}), meas_p_a))
    return real_p

print(real_p_a)
print(meas_p_a)
print(triangulate(t_ab, meas_p_a, meas_p_b, z1, z2))