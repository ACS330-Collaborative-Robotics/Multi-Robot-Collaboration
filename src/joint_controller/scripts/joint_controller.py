#!/usr/bin/env python

# Name: Joint Controller
# Author: Tom Richards (tmrichards1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Float64
from custom_msgs.msg import Joints
from sys import argv

## TODO:
# Add details to readMe

def main():
    rospy.init_node('joint_controller')
    # Toggles for Physical and Simulation robots
    # TODO: Setup topics and subscribers to control these variables
    global enablePhysical
    enablePhysical = True
    global enableSimulation
    enableSimulation = True
    #Getting robot name pryamiter
    global robot_name

    robot_name =  "/" + argv[1]
    print("----------------------------------")
    print(robot_name + " Initialised.")
    print("----------------------------------")

    # Setup subscriber for Joint Angle demands
    rospy.Subscriber(robot_name + "/joint_angles", Joints, callback)
    #TODO: Add multirobot support

    # Wait until a callback happens
    rospy.spin()

def callback(data):
    global joint_angles
    joint_angles = list(data.joints)
    #rospy.loginfo("Angles Recived: %s", joint_angles)

    # Telling Physical to move
    if enablePhysical:
        pubPhysical = rospy.Publisher(robot_name + "/physical/joint_angles", Joints, queue_size=10)
        #rospy.loginfo("Angles Published to physical: %s", joint_angles)
        pubPhysical.publish(joint_angles)
            
    # Telling Simulation to move
    if enableSimulation:
        for joint_num in range(len(joint_angles)):
            pubSimulation = rospy.Publisher(robot_name + "/joint" + str(joint_num) + "_position_controller/command", Float64, queue_size=10)
            pubSimulation.publish(joint_angles[joint_num])

    # Joint lims in radians [[-2.269,2.269],[-0.873,1.047],[-1.920,1.309],[-2.443,2.443],[-1.221,1.047],[-2.094,2.094]]
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass