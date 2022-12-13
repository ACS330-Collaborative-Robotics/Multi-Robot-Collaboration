#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import random
import math
from basic_movement.msg import Joints

def talker():
    # Shouting where i want the robot to move to then changeing to diffent position after getting there
    # Joint lims in radians [[-2.269,2.269],[-0.873,1.047],[-1.920,1.309],[-2.443,2.443],[-1.221,1.047],[-2.094,2.094]]
    pub = rospy.Publisher('/mover6_a/physical/joint_angles', Joints, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.1) # 10hz
    while not rospy.is_shutdown():
        joint_angles = posistions = [round(random.uniform(-2.269,2.269), 3), round(random.uniform(-0.873,1.047), 3), round(random.uniform(-1.920,1.309), 3), round(random.uniform(-2.443,2.443), 3), round(random.uniform(-1.221,1.047), 3), round(random.uniform(-2.094,2.094), 3)]
        rospy.loginfo(joint_angles)
        pub.publish(joint_angles)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
