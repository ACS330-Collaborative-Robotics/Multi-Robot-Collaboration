#!/usr/bin/env python

# Name: Gripper Control Test Script
# Author: Tom Richards (tmrichards1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('mover6_a_p/gripper_state', Bool, queue_size=10)
    rospy.init_node('Gripper_ControlA')

    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        val = input("SetGripperPossition ->")
        if val == "True":
            temp = True
        else:
            temp = False

        pub.publish(temp)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
