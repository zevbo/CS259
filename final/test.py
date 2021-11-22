from numpy.lib.function_base import angle
from ik import *
import numpy as np

thetas1 = np.array([1] * 6)
thetas2 = np.array([-0.9] * 6)
get_thetas_persistent(calc_tsb(thetas1), thetas2)

print("Minimum Possible: ")
diff = thetas2 - thetas1
normalize_vec(diff)
print(angle_cost(diff))