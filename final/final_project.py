from triangulization import triangulate
from stuff import *
from moveTo import *
from grip import *
from camera import *
from position import *
from findObject import find_obj, find_dest

import rosnode
import rospy
import time
import random
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sympy import symbols, solve, Eq, linsolve, core

armCmd = rospy.Publisher(
    '/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)

# maybe can get image data from here: http://wiki.ros.org/usb_cam
# how to fix failure of gripper and camera: https://github.com/ros-industrial/robotiq/issues/118

# pointing down
r_searching_1 = np.array([
    [-1, 0, 0],
    [0, 1, 0],
    [0, 0, -1],
])
# rotated a little bit from downards pointing
r_searching_2 = np.dot(x_rotation(math.pi / 6), r_searching_1)
search_range = [-600, -100], [-500, 500], [150, 350]


def random_search_pos():
    return np.array(list(map(lambda range: random.uniform(*range), search_range)))


def random_t_goal(r_searching):
    return r_and_shift_to_t(r_searching, random_search_pos())


def move_to_random(r_searching):
    move_to(random_t_goal(r_searching))


def search_for_with(search_f, r_searching):
    while True:
        move_to_random(r_searching)
        img = None  # get_image()
        loc = search_f(img)
        if loc != None:
            img_x, img_y = loc
            # ratio = tan(angle)
            # max_pos = z_dist * tan(max_angle)
            # TODO: make sure the x and y of the camera are oriented correctly
            r_x = math.tan(maxHorAngle) * img_x / maxX
            r_y = math.tan(maxVerAngle) * img_y / maxY
            # currT()
            return np.transpose(np.array([r_x, r_y, 1])), np.array([])


def locate(search_f):
    pos1, t_s1 = search_for_with(search_f, r_searching_1)
    pos2, t_s2 = search_for_with(search_f, r_searching_2)
    z1, z2 = symbols('z1, z2')
    # t_ab, meas_p_a, meas_p_b, z1, z2
    t_ab = np.dot(np.linalg.inv(t_s1), t_s2)
    return triangulate(t_ab, pos1, pos2, z1, z2)


def move_to_find(search_f):
    move_to(locate(search_f))


def pick_up():
    move_to_find(find_obj)
    grip()


def deposit():
    move_to_find(find_dest)
    release()
