import time
from ik import get_thetas_persistent
from position import curr_thetas
from fk import calc_tsb
import params

import rosnode
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# note: this will change
interval = 5
clearance = 1

def move_to(t_goal):
    
    # rospy.init_node('moving')

    topic = 'arm_controller/command' if params.on_gazebo else '/scaled_pos_joint_traj_controller/command'
    armCmd = rospy.Publisher(topic, JointTrajectory, queue_size=10)

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

    print("sending arm command")

    while ticks < 20:
        armCmd.publish(testMsg)
        rate.sleep()
        ticks += 1

    time.sleep(interval + clearance)

def moveJoints(thetas):
    move_to(calc_tsb(thetas))
