#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import random

def talker():
    pub = rospy.Publisher('/mover6/joint1_position_controller/command', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():
        pos = random.randint(-2,2)
        rospy.loginfo(pos)
        pub.publish(pos)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass