#!/usr/bin/env python

# Name: Gripper Control Test Script
# Author: Tom Richards (tmrichards1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('mover6_b_p/gripper_state', Bool, queue_size=10)
    pub_s = rospy.Publisher('mover6_b/gripper_state', Bool, queue_size=10)
    rospy.init_node('Gripper_ControlB')

    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        val = input("SetGripperPossition ->")
        if val == "True":
            temp = True
        else:
            temp = False

        pub.publish(temp)
        pub_s.publish(temp)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
