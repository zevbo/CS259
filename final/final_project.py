from triangulization import triangulate, triangulate2
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
angle2 = math.pi / 9
r_searching_2 = matmul(x_rotation(math.pi / 35),
                       y_rotation(angle2), r_searching_1)
angle3 = math.pi / 4
r_searching_3 = matmul(x_rotation(math.pi / -35),
                       y_rotation(angle3), r_searching_1)
search_range1 = [-500, -325], [0, 600], [200, 450]
search_range2 = [-325, -75], [0, 600], [200, 450]


def random_search_pos(search_range):
    return np.array(list(map(lambda range: random.uniform(*range), search_range)))


def random_t_goal(r_searching, search_range):
    return r_and_shift_to_t(r_searching, random_search_pos(search_range))


def move_to_random(r_searching, search_range):
    return move_to(random_t_goal(r_searching, search_range), extra_legality=lambda angles: angles[1] < - math.pi / 4)


dest_spottings = []


def full_find(search_f, z, file="found.jpg"):
    img = getImage()
    result = search_f(img)
    if result != None:
        loc, big_contour = result
        x, y, w, h = cv2.boundingRect(big_contour)
        disp_image = cv2.rectangle(
            img, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.imwrite(file, disp_image)
        img_x, img_y = loc
        # ratio = tan(angle)
        # max_pos = z_dist * tan(max_angle)
        # TODO: make sure the x and y of the camera are oriented correctly
        r_x = math.tan(maxHorAngle) * img_x / maxX
        r_y = math.tan(maxVerAngle) * img_y / maxY
        # r_y and r_x here switched because that's how the real coordinates are
        return np.transpose(np.array([[-r_y * z, r_x * z, z, 1]])), curr_t()
    else:
        return False


def search_for_with(search_f, r_searching, search_range, z):
    while True:
        found = move_to_random(r_searching, search_range)
        if not found:
            continue
        time.sleep(1)
        getImage()
        time.sleep(12)

        dest_search = full_find(find_dest, 1, file="found_dest.jpg")
        if type(dest_search) != bool:
            dest_spottings.append(dest_search)

        res = full_find(search_f, z)
        if type(res) != bool:
            return res


obj_height = -180 - gripper_length
dest_height = -225 - gripper_length


def add_z(result, z):
    pos, t = result
    new_pos = np.transpose(
        np.array([[pos[0][0] * z, pos[1][0] * z, pos[2][0] * z, 1]]))
    return new_pos, t


def locate(search_f, height, use_dest_spottings):
    z1, z2 = symbols('z1, z2')
    pos1, t_sb1 = add_z(dest_spottings[0], z1) if use_dest_spottings and len(
        dest_spottings) > 0 else search_for_with(search_f, r_searching_2, search_range1, z1)
    pos2, t_sb2 = add_z(dest_spottings[1], z2) if use_dest_spottings and len(
        dest_spottings) > 1 else search_for_with(search_f, r_searching_3, search_range2, z2)
    # t_ab, meas_p_a, meas_p_b, z1, z2
    t_sc1 = np.dot(t_sb1, t_bc)
    t_sc2 = np.dot(t_sb2, t_bc)
    t_ab = np.dot(np.linalg.inv(t_sc1), t_sc2)
    # p = triangulate(t_sc1, pos1, z1, height)
    p = triangulate2(t_sc1, t_ab, pos1, pos2, z1, z2)
    return r_and_shift_to_t(r_searching_1, p)


clearance = 70


def move_to_find(search_f, height, extra, use_dest_spottings):
    # okay buddy. Can't pass a point to move_to
    t_goal = locate(search_f, height, use_dest_spottings)
    t_goal[2][3] += extra
    t_intermdiate = np.array(t_goal)
    t_intermdiate[2][3] += clearance
    move_to_persistent(t_intermdiate)
    move_to_persistent(t_goal, interval=2)


def pick_up():
    release()
    move_to_find(find_obj, obj_height, -25, False)
    grip()
    time.sleep(1)


def deposit():
    move_to_find(find_dest, dest_height, 70, True)
    release()
    time.sleep(1)
    t = curr_t()
    t[2][3] += clearance
    move_to_persistent(t, interval=2)
    move_to_persistent(
        calc_tsb(np.array([0, -math.pi / 2, 0, 0, -math.pi / 2, 0])))
