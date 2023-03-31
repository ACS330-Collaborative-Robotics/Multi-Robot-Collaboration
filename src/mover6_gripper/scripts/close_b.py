#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

gripper_a = -0.5
gripper_b = -0.5

def talker_a(event):
    pub1.publish(gripper_a)


def talker_b(event):
    pub2.publish(gripper_b)

if __name__ == '__main__':
    rospy.init_node('grippers', anonymous=True)
    pub1 = rospy.Publisher('/mover6_b/jointgripper_a_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/mover6_b/jointgripper_b_position_controller/command', Float64, queue_size=10)
    rospy.Timer(rospy.Duration(0.1),talker_a)
    rospy.Timer(rospy.Duration(0.1),talker_b)
    rospy.spin()