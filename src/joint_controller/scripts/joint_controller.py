#!/usr/bin/env python

# Name: Joint Controller
# Author: Tom Richards (tmrichards1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from custom_msgs.msg import Joints
from sys import argv

global enablePhysical
enablePhysical = False
global enableSimulation
enableSimulation = False

def main():
    rospy.init_node('joint_controller')

    # Get robot name parameter
    global robot_name
    robot_name =  "/" + argv[1]
    
    rospy.loginfo("%s Joint Controller Initialised.\n", robot_name)

    rospy.Subscriber(robot_name + "/pause_physical", Bool, callback_specific_robot_stop)

    # Setup subscriber for Joint Angle demand
    # Telling Physical to move
    rospy.Subscriber(robot_name + "/joint_angles", Joints, callback_physical)

            
    # Telling Simulation to move
    rospy.Subscriber(robot_name + "/joint_angles", Joints, callback_sim)

    # Setup joint controller to be a publisher for the required topics
    for joint_num in range(6):
        pubSimulation = rospy.Publisher(robot_name + "/joint" + str(joint_num+1) + "_position_controller/command", Float64, queue_size=10)
    pubPhysical = rospy.Publisher(robot_name + "_p/physical/joint_angles", Joints, queue_size=10)

    # Wait until a callback happens
    rospy.spin()

def callback_sim(data):
    if not enableSimulation:
        global joint_angles
        joint_angles = list(data.joints)
        #rospy.loginfo("Angles Recived: %s", joint_angles)
        for joint_num in range(len(joint_angles)):
            pubSimulation = rospy.Publisher(robot_name + "/joint" + str(joint_num+1) + "_position_controller/command", Float64, queue_size=10)
            pubSimulation.publish(joint_angles[joint_num])
        

    # Joint lims in radians [[-2.269,2.269],[-0.873,1.047],[-1.920,1.309],[-2.443,2.443],[-1.221,1.047],[-2.094,2.094]]

def callback_specific_robot_stop(data):
    global enablePhysical
    enablePhysical = not bool(data.data)


def callback_physical(data):
    if not enablePhysical:
        global joint_angles
        joint_angles = list(data.joints)
        pubPhysical = rospy.Publisher(robot_name + "_p/physical/joint_angles", Joints, queue_size=10)
        #rospy.loginfo("Angles Published to physical: %s", joint_angles)
        pubPhysical.publish(joint_angles)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass