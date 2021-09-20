import rosnode
import rospy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('prelab1')

armCmd = rospy.Publisher('/arm_controller/command',JointTrajectory,queue_size=10)

def test(secs):
    p = JointTrajectoryPoint()
    p.positions = [1,-1,-1,-1,-1,1]
    p.velocities = [0,0,0,0,0,0]
    p.time_from_start.secs = 10

    testMsg = JointTrajectory()
    testMsg.points = [p]
    testMsg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


    print(testMsg)
    armCmd.publish(testMsg)

    time.sleep(secs)

test(60)
