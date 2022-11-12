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

    limits = [[-130, 130], [-50, 60], [-110, 75], [-140, 140], [-70, 60], [-120, 120]]
    points = 9

    rospy.init_node('talker', anonymous=True)
    T_period = 2
    rate = rospy.Rate(1/T_period)

    currentPoint = 0

    while not rospy.is_shutdown():
        for joint in range(6):
            pos = (currentPoint*(limits[joint][1] - limits[joint][0])/(points-1) + limits[joint][0])*3.14/180

            rospy.loginfo("Joint no: " + str(joint+1) + " to " + str(round(pos, 2)))
            pub[joint].publish(pos)

        currentPoint = (currentPoint + 1) % points

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass