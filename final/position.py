
import rosnode
import rospy
from sensor_msgs.msg import JointState

from fk import calc_tsb
from specs import *

def curr_data():
    return rospy.wait_for_message('/joint_states', JointState)

def curr_thetas():
    data = curr_data()
    theta_dict = dict(zip(data.name, data.position))
    return [theta_dict[joint] for joint in joint_order]

def curr_v():
    return curr_data().velocity

def curr_t():
    return calc_tsb(curr_thetas())
