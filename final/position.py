
import rosnode
import rospy
from sensor_msgs.msg import JointState

from fk import calc_tsb

def curr_data():
    
    rospy.init_node('listener', anonymous=True)

    return rospy.wait_for_message('/joint_states', JointState)

def curr_thetas():
    return list(curr_data().position)

def curr_v():
    return curr_data().velocity

def curr_t():
    return calc_tsb(curr_thetas())
