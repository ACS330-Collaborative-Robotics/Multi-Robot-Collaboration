#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import random

def talker():
    pub = [rospy.Publisher('/mover6/joint1_position_controller/command', Float64, queue_size=10),
            rospy.Publisher('/mover6/joint2_position_controller/command', Float64, queue_size=10),
            rospy.Publisher('/mover6/joint3_position_controller/command', Float64, queue_size=10),
            rospy.Publisher('/mover6/joint4_position_controller/command', Float64, queue_size=10),
            rospy.Publisher('/mover6/joint5_position_controller/command', Float64, queue_size=10),
            rospy.Publisher('/mover6/joint6_position_controller/command', Float64, queue_size=10)]

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5) # T = 2s

    while not rospy.is_shutdown():
        pos = random.random()*4 - 2
        rospy.loginfo(pos)
        pub[0].publish(pos)
        
        pos = 0;
        for i in range(1, 6):
            rospy.loginfo(pos)
            pub[i].publish(pos)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass