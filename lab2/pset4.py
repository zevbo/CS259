from stuff import *
import sympy as sym


def e511():
    l = sym.Symbol("L")
    b3 = trap_arr([0, -1, 0, 0, 0, 0])
    b2 = trap_arr([0, -1, 0, 2 * l, 0, 0])
    b1 = trap_arr([0, 0, 1, 2 * l, 0, -1 * l])
    # Part a
    j_a = calc_body_j([0, math.pi / 4, -math.pi/4], [b1, b2, b3])
    j_a_pos = j_a[3:6]
    print("DEBUG: ")
    print(j_a)
    print(j_a_pos)
    print("Part A: ")
    desired = np.array([10, 0, 0])
    print(np.dot(np.linalg.inv(j_a_pos), desired))
