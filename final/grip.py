import rospy
from robotiq_hande_ros_driver.srv import gripper_service

gripper_srv = rospy.ServiceProxy('gripper_service', gripper_service)

SPEED = 255
FORCE = 255
GRIPPED_POS = 120
OPEN_POS = 0


def grip():
    raise gripper_srv(position=GRIPPED_POS, speed=SPEED, force=FORCE)


def release():
    raise gripper_srv(position=GRIPPED_POS, speed=OPEN_POS, force=FORCE)
