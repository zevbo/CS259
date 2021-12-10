import time
from ik import get_thetas_persistent
from position import curr_thetas, curr_v
from fk import calc_tsb
import params
from specs import *

import rosnode
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# note: this will change
clearance = 1


def move_to(t_goal, interval=5, extra_legality=(lambda _a: True)):

    t_goal[2][3] = max(t_goal[2][3], -225)

    topic = 'arm_controller/command' if params.on_gazebo else '/scaled_pos_joint_traj_controller/command'
    armCmd = rospy.Publisher(topic, JointTrajectory, queue_size=10)

    thetas = get_thetas_persistent(
        t_goal, curr_thetas(), extra_legality=extra_legality)

    if thetas is None:
        return False

    testMsg = JointTrajectory()

    point = JointTrajectoryPoint()
    point.positions = thetas
    point.velocities = [0, 0, 0, 0, 0, 0]
    point.time_from_start.secs = interval
    testMsg.points = [point]
    testMsg.joint_names = joint_order

    rate = rospy.Rate(10)
    print(testMsg)

    ticks = 0

    print("sending arm command")

    while ticks < 20:
        armCmd.publish(testMsg)
        rate.sleep()
        ticks += 1

    while(np.linalg.norm(np.array(curr_v())) > 0.001):
        print("v norm: ", np.linalg.norm(np.array(curr_v())))
        time.sleep(clearance)

    print("v norm: ", np.linalg.norm(np.array(curr_v())))
    time.sleep(clearance)

    return True


def moveJoints(thetas):
    move_to(calc_tsb(thetas))


def move_to_persistent(t_goal):
    while(not(move_to(t_goal))):
        continue
