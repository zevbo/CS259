import time
from ik import get_thetas_persistent
from position import *

t_goal = curr_t()
thetas = get_thetas_persistent(t_goal, curr_thetas())

print(thetas)
