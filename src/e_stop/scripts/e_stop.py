#!/usr/bin/env python

# Name: Emergency Stop Topic
# Author: Tom Richards (tmrichards1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Bool
from sys import argv

global phicGripperEStop
global phicGripper
global simGripper
global phicRobot
global simRobot
phicGripperEStop = False
phicGripper = False
simGripper = False
phicRobot = False
simRobot = False
mover6a = False
mover6b = False

def main():
    rospy.init_node('e_stop')

    phicGripperEStoppib = rospy.Publisher(, , queue_size=10)
    simGrippub = rospy.Publisher(, , queue_size=10)
    simRobopub = rospy.Publisher(, , queue_size=10)
    PhiscGrippub = rospy.Publisher(, , queue_size=10)
    pisicRobopub = rospy.Publisher(, , queue_size=10)
    mover6apub = rospy.Publisher(, , queue_size=10)
    mover6bpub = rospy.Publisher(, , queue_size=10)



    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        

        rospy.Subscriber(, , callback)

        rate.sleep()

def callback(data):
    global 
    
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass