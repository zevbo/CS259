from stuff import *
from sympy import symbols, solve, Eq, linsolve
import numpy as np

z1 = symbols('z1')
z2 = symbols('z2')

test_p_a = np.transpose(np.array([0, 0, z1, 1]))
test_p_b = np.transpose(np.array([0, 0, z2, 1]))

test_t_ab = np.array([
    [0, 0, -1, 3],
    [0, 1, 0, 0],
    [1, 0, 0, 4],
    [0, 0, 0, 1]
])

p_a2 = np.dot(test_t_ab, test_p_b)

zero_p = p_a2 - test_p_a
print(zero_p)
solution = linsolve(zero_p.tolist(), (z1, z2))
print(solution)