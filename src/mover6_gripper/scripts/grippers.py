#!/usr/bin/env python
# license removed for brevity
import rospy
from mover6_gripper.msg import grip

def talker():
    pub = rospy.Publisher('control', grip, queue_size=10)
    rospy.init_node('grippers', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass