#!/usr/bin/env python

# Name: Gripper Controller
# Author: Tom Richards (tmrichards1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Bool, Float64
from cpr_robot.msg import ChannelStates
from sys import argv

global activateGripper
activateGripper = None

def main():
    rospy.init_node('gripper_controller')

    enableGripper = True #TODO: Add enable/disable functionality

    global robot_name
    robot_name =  "/" + argv[1]
    print("----------------------------------")
    print(robot_name + " Gripper Initialised.")
    print("----------------------------------")
    
    # Gripper must be published at 10 Hz constantly otherwise it will not move the gripper
    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        pubGripper = rospy.Publisher(robot_name + '_p/OutputChannels', ChannelStates, queue_size=10)

        rospy.Subscriber(robot_name + "/gripper_state", Bool, callback_gripper)

        gripperstate = ChannelStates()
        gripperstate.Header.stamp = rospy.get_rostime()

        if activateGripper == True:
            gripperstate.state = [False, False, False, False, True, True]
            rospy.loginfo(robot_name + "Gripper Open")

        if activateGripper == False:
            gripperstate.state = [False, False, False, False, False, True]
            rospy.loginfo(robot_name + "Gripper Closed")

        pubGripper.publish(gripperstate)
        rate.sleep()

        global gripper_a, gripper_b

        if activateGripper == True:
            gripper_a = 1
            gripper_b = 1
            rospy.loginfo(robot_name + "Gripper Open")

        if activateGripper == False:
            gripper_a = -0.5
            gripper_b = -0.5
            rospy.loginfo(robot_name + "Gripper Closed")
        
        grip_letter = ['a', 'b']
        
        for grip_num in range(2):
            pubGripper_s = rospy.Publisher(robot_name + "/jointgripper_" + grip_letter[grip_num] + "_position_controller/command", Float64, queue_size=10)
        rospy.spin()


def callback_gripper(data):
    global activateGripper
    activateGripper = data.data

def callback_gripper_s(data):
    global grip_angles
    grip_letter = ['a', 'b']
    grip_angles = list(data.gripper)
    for grip_num in range(len(grip_angles)):
        pubGripper_s = rospy.Publisher(robot_name + "/jointgripper_" + grip_letter[grip_num] + "_position_controller/command", Float64, queue_size=10)
        pubGripper_s.publish(grip_angles[grip_num])

        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass