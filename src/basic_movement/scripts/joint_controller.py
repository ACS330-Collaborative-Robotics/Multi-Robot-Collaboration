#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import random
import math
from basic_movement.msg import Joints

enablePhiscal = True
enableSIM = False


## TODO:
# Talk to sim robot
# Recive angles from inv_kin_srv.py - see photo from for topic names
# Add details to readMe
# Better comments
# movement demo, pass string when lauching in lauch file
# Set arms Acuracy
# S et sim acuracy
# set overall speed
# check acuracy and gairns are complatable
# https://answers.ros.org/question/290992/parallel-execution-of-same-ros-nodes/

def main():
    rospy.init_node('joint_controller', anonymous=True)
    ##recive angles
    rate = rospy.Rate(0.1) # 10hz
    while not rospy.is_shutdown():
        #subsibers for phiscal and sim
        global enablePhiscal
        enablePhiscal = True
        global enableSIM
        enableSIM = False
        #
        rospy.Subscriber("/mover6_a/joint_angles", Joints, callback)
        ## ros loop
        rate.sleep()
        rospy.spin()

def callback(data):
    global joint_angles
    joint_angles = list(data.joints)
    rospy.loginfo("Angles Recived: %s", joint_angles)

    # Telling phiscail to move
    if enablePhiscal:
        pubPhisic = rospy.Publisher('/mover6_a/physical/joint_angles', Joints, queue_size=10)
        rospy.loginfo("Angles Published to phical: %s", joint_angles)
        pubPhisic.publish(joint_angles)
            
        
    # Telling the Sim to move
    if enableSIM:
        for joint_num in range(len(joint_angles)):
            pubSim = rospy.Publisher("/mover6_a/joint" + str(joint_num) + "_position_controller/command", Float64, queue_size=10)
            pubSim.publish(joint_angles[joint_num])


    # Joint lims in radians [[-2.269,2.269],[-0.873,1.047],[-1.920,1.309],[-2.443,2.443],[-1.221,1.047],[-2.094,2.094]]
        

        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
