import rosnode
import rospy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('prelab1')

armCmd = rospy.Publisher('/scaled_pos_joint_traj_controller/command',JointTrajectory,queue_size=10)

def run():
    def make_points(interval, positions):
        points = []
        time = 0
        for pos in positions:
            time += interval
            p = JointTrajectoryPoint()
            p.positions = pos
            p.velocities = [0,0,0,0,0,0]
            p.time_from_start.secs = time
            points.append(p)
        return points

    testMsg = JointTrajectory()
    positions = [[0,0,0,0,0,0],[-1,0,0,0,0,0],[-1,-1,0,0,0,0],[-1,-1,2,0,0,0],[-1,-1,2,3,0,0]]
    testMsg.points = make_points(3, positions)
    testMsg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    rate = rospy.Rate(10)
    print(testMsg)
   
    ticks = 0
    while ticks < 20:
        armCmd.publish(testMsg)
        rate.sleep()
        ticks += 1

   # time.sleep(secs)

run()
