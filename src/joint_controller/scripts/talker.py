#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from cpr_robot.msg import ChannelStates

def talker():
    pub = rospy.Publisher('/mover6_a_p/OutputChannels', ChannelStates, queue_size=10)
    rospy.init_node('Gripper_Control')
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        temp = ChannelStates()
        temp.Header.stamp = rospy.get_rostime()
        temp.state = [False, False, False, False, True, True]
        pub.publish(temp)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
