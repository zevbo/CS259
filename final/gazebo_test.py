import moveTo
import numpy as np 
import rospy

rospy.init_node("gazebo_test")
moveTo.moveJoints(np.array([0.5, -1.0, 0.2, 1, 1, 0]))
