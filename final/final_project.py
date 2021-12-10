from triangulization import triangulate
from stuff import *
from moveTo import *
from grip import *
from camera import *
from position import *
from fk import calc_tsb
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
    [1, 0, 0],
    [0, -1, 0],
    [0, 0, -1],
])
# rotated a little bit from downards pointing
angle = 0  # math.pi / 6
r_searching_2 = np.dot(y_rotation(angle), r_searching_1)
search_range = [-500, -175], [0, 600], [200, 450]


def random_search_pos():
    return np.array(list(map(lambda range: random.uniform(*range), search_range)))


def random_t_goal(r_searching):
    return r_and_shift_to_t(r_searching, random_search_pos())


def move_to_random(r_searching):
    return move_to(random_t_goal(r_searching))


def search_for_with(search_f, r_searching, z):
    while True:
        found = move_to_random(r_searching)
        if not found:
            continue
        time.sleep(1)
        getImage()
        time.sleep(12)
        img = getImage()
        result = search_f(img)
        if result != None:
            loc, big_contour = result
            x, y, w, h = cv2.boundingRect(big_contour)
            disp_image = cv2.rectangle(
                img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.imwrite("found.jpg", disp_image)
            img_x, img_y = loc
            # ratio = tan(angle)
            # max_pos = z_dist * tan(max_angle)
            # TODO: make sure the x and y of the camera are oriented correctly
            r_x = math.tan(maxHorAngle) * img_x / maxX
            r_y = math.tan(maxVerAngle) * img_y / maxY
            # r_y and r_x here switched because that's how the real coordinates are
            return np.transpose(np.array([[-r_y * z, r_x * z, z, 1]])), curr_t()


obj_height = -180 - gripper_length
dest_height = -225 - gripper_length


def locate(search_f, height):
    z1, z2 = symbols('z1, z2')
    pos1, t_sb1 = search_for_with(search_f, r_searching_2, z1)
    # pos2, t_sb2 = search_for_with(search_f, r_searching_2)
    # t_ab, meas_p_a, meas_p_b, z1, z2
    t_sc1 = np.dot(t_sb1, t_bc)
    print("t_sc1: ")
    print(t_sc1)
    # t_sc2 = np.dot(t_sb2, t_bc)
    # t_ab = np.dot(np.linalg.inv(t_sc1), t_sc2)
    p = triangulate(t_sc1, pos1, z1, height)
    return r_and_shift_to_t(r_searching_1, p)


clearance = 40


def move_to_find(search_f, height, extra):
    # okay buddy. Can't pass a point to move_to
    t_goal = locate(search_f, height)
    t_goal[2][3] += extra
    t_intermdiate = np.array(t_goal)
    t_intermdiate[2][3] += clearance
    move_to(t_intermdiate)
    move_to(t_goal, interval=2)


def pick_up():
    release()
    move_to_find(find_obj, obj_height, -20)
    grip()
    time.sleep(1)


def deposit():
    move_to_find(find_dest, dest_height, 50)
    release()
    time.sleep(1)
    t = curr_t()
    t[2][3] += clearance
    move_to(t, interval=2)
    move_to(calc_tsb(np.array([0, -math.pi / 2, 0, 0, -math.pi / 2, 0])))
