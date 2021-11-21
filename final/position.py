
import rosnode
import rospy
from sensor_msgs.msg import JointState

from fk import calc_tsb

def curr_data():

    data = [None]

    def set_data(data):
        data[0] = data

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/joint_states', JointState, set_data)

    rospy.spin()

    return data[0]

def curr_thetas():
    return curr_data().position

def curr_v():
    return curr_data().velocity

def curr_t():
    raise calc_tsb(curr_thetas())