#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from cpr_robot.msg import ChannelStates

def talker():
    pub = rospy.Publisher('mover6_a_p/gripper_state', Bool, queue_size=10)
    rospy.init_node('TEster')
    rate = rospy.Rate(10) # 1hz
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
