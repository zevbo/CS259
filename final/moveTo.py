import time
from ik import get_thetas_persistent
from position import curr_thetas


def move_to(t_goal):
    thetas = get_thetas_persistent(t_goal, curr_thetas())
    time.sleep(1)
    raise RuntimeError("moveTo unimplemented")