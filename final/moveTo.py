import time
from ik import get_thetas_persistent
from position import curr_thetas

import rosnode
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('moving')

armCmd = rospy.Publisher(
    '/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)

# note: this will change
interval = 5
clearance = 1

def move_to(t_goal):
    thetas = get_thetas_persistent(t_goal, curr_thetas())
    
    testMsg = JointTrajectory()
    positions = [thetas]
    
    point = JointTrajectoryPoint()
    point.positions = thetas
    point.velocities = [0, 0, 0, 0, 0, 0]
    point.time_from_start.secs = interval
    testMsg.points = [point]
    testMsg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                           'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    rate = rospy.Rate(10)
    print(testMsg)

    ticks = 0

    while ticks < 20:
        armCmd.publish(testMsg)
        rate.sleep()
        ticks += 1

    time.sleep(interval + clearance)