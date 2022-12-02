#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64MultiArray

def talker():
    pub = rospy.Publisher('chatter2', Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    corner_a1 = [-0.5, -0.5]
    corner_a2 = [0.5, 0]
    corner_a3 = [0, 0.5]
    corner_a4 = [0.5, 0.5]
    corner_b1 = [0, 0.5]
    corner_b2 = [-0.5, 0.5]
    corner_b3 = [0.5, -0.5]
    corner_b4 = [0.5, 0]
    mover6_a_zone = Float64MultiArray()
    mover6_b_zone = Float64MultiArray()
    mover6_a_zone.data = [corner_a1, corner_a2, corner_a3, corner_a4]
    mover6_b_zone.data = [corner_b1, corner_b2, corner_b3, corner_b4]
   
    pub.publish(mover6_a_zone)
    pub.publish(mover6_b_zone)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
