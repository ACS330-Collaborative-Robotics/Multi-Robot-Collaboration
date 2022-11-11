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
    rate = rospy.Rate(1/10)

    while not rospy.is_shutdown():
        for i in range(6):
            pos = random.randint(-180, 180) * 3.14/180
            rospy.loginfo("Joint no: " + str(i+1) + " to " + str(pos))
            pub[i].publish(pos)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass