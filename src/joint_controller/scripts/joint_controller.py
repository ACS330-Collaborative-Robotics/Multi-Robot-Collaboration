#!/usr/bin/env python

# Name: Joint Controller
# Author: Tom Richards (tmrichards1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from custom_msgs.msg import Joints
from sys import argv

global e_stop
e_stop = False

## TODO:
# Add details to readMe

def main():
    rospy.init_node('joint_controller')
    # Toggles for Physical and Simulation robots
    # TODO: Setup topics and subscribers to control these variables
    enablePhysical = True
    enableSimulation = True
    #Getting robot name pryamiter
    global robot_name
    robot_name =  "/" + argv[1]
    print("----------------------------------")
    print(robot_name + " Joints Initialised.")
    print("----------------------------------")

    # Setup subscriber for Joint Angle demand
    # Telling Physical to move
    if enablePhysical:
        rospy.Subscriber(robot_name + "/e_stop", Bool, callback_spec_robo_stop)
        if e_stop == False:
            #print("The estop is  "+e_stop)
            rospy.Subscriber(robot_name + "/joint_angles", Joints, callback_physical)
        else:
            print("E_stop TRue")
        
            
    # Telling Simulation to move
    if enableSimulation:
        rospy.Subscriber(robot_name + "/joint_angles", Joints, callback_sim)

    # Setup joint controller to be a publisher for the required topics
    for joint_num in range(6):
        pubSimulation = rospy.Publisher(robot_name + "/joint" + str(joint_num+1) + "_position_controller/command", Float64, queue_size=10)
    pubPhysical = rospy.Publisher(robot_name + "_p/physical/joint_angles", Joints, queue_size=10)

    # Wait until a callback happens
    rospy.spin()

def callback_sim(data):
    global joint_angles
    joint_angles = list(data.joints)
    #rospy.loginfo("Angles Recived: %s", joint_angles)
    for joint_num in range(len(joint_angles)):
        pubSimulation = rospy.Publisher(robot_name + "/joint" + str(joint_num+1) + "_position_controller/command", Float64, queue_size=10)
        pubSimulation.publish(joint_angles[joint_num])
        

    # Joint lims in radians [[-2.269,2.269],[-0.873,1.047],[-1.920,1.309],[-2.443,2.443],[-1.221,1.047],[-2.094,2.094]]

def callback_spec_robo_stop(data):
    global e_stop
    e_stop = data.data


def callback_physical(data):
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