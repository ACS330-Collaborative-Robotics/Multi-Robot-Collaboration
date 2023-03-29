#!/usr/bin/env python

# Name: Gripper Controller
# Author: Tom Richards (tmrichards1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Bool, Float64
from cpr_robot.msg import ChannelStates
from sys import argv

global activateGripper
activateGripper = None

global e_stop
e_stop = False

def main():
    rospy.init_node('gripper_controller')

    enableGripper = True #TODO: Add enable/disable functionality

    global robot_name
    robot_name =  "/" + argv[1]
    
    rospy.loginfo("%s Gripper Initialised.\n", robot_name)

    # Gripper must be published at 10 Hz constantly otherwise it will not move the gripper
    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        pubGripper_physical = rospy.Publisher(robot_name + '_p/OutputChannels', ChannelStates, queue_size=10)

        gripper_arm_letters = ['a', 'b']
        physical_gripper_angles = [0, 0]

        pubGripper_simulation = []
        for grip_num in range(len(physical_gripper_angles)):
            pubGripper_simulation.append(rospy.Publisher(robot_name + "/jointgripper_" + gripper_arm_letters[grip_num] + "_position_controller/command", Float64, queue_size=10))

        rospy.Subscriber(robot_name + "_p/gripper_state", Bool, callback_gripper)
        rospy.Subscriber("/emergency_stop", Bool, callback_emergency_stop)
        rospy.Subscriber(robot_name + "/e_stop", Bool, callback_gripper_robo_spec)

        gripperstate = ChannelStates()
        gripperstate.Header.stamp = rospy.get_rostime()

        if e_stop == True:
            gripperstate.state = [False, False, False, False, False, False]
            rospy.logdebug(robot_name + " Gripper Emergency Stopped")
            
        elif activateGripper == True:
            gripperstate.state = [False, False, False, False, True, True]
            physical_gripper_angles = [1, 1]
            rospy.logdebug(robot_name + " Gripper Open")

        elif activateGripper == False:
            gripperstate.state = [False, False, False, False, False, True]
            physical_gripper_angles = [-0.5, -0.5]
            rospy.logdebug(robot_name + " Gripper Closed")

        for grip_num in range(len(physical_gripper_angles)):
            pubGripper_simulation = rospy.Publisher(robot_name + "/jointgripper_" + gripper_arm_letters[grip_num] + "_position_controller/command", Float64, queue_size=10)
            pubGripper_simulation.publish(physical_gripper_angles[grip_num])

        pubGripper_physical.publish(gripperstate)

        rate.sleep()

def callback_gripper(data):
    global activateGripper
    activateGripper = data.data

def callback_gripper_robo_spec(data):
    global activateGripper
    activateGripper = data.data

def callback_emergency_stop(data):
    global e_stop
    e_stop = data.data
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass