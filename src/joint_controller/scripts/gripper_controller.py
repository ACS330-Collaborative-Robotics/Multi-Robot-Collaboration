#!/usr/bin/env python

# Name: Gripper Controller
# Author: Tom Richards (tmrichards1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Bool
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
    print("----------------------------------")
    print(robot_name + " Gripper Initialised.")
    print("----------------------------------")
    
    # Gripper must be published at 10 Hz constantly otherwise it will not move the gripper
    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        pubGripper = rospy.Publisher(robot_name + '_p/OutputChannels', ChannelStates, queue_size=10)

        rospy.Subscriber(robot_name + "_p/gripper_state", Bool, callback_gripper)
        rospy.Subscriber("/emergency_stop", Bool, callback_emergency_stop)

        # TODO: Add physical pause to gripper per robot

        gripperstate = ChannelStates()
        gripperstate.Header.stamp = rospy.get_rostime()

        if e_stop == True:
            gripperstate.state = [False, False, False, False, False, False]
            #rospy.loginfo(robot_name + "Gripper Stopped")
            
        elif activateGripper == True:
            gripperstate.state = [False, False, False, False, True, True]
            #rospy.loginfo(robot_name + "Gripper Open")

        elif activateGripper == False:
            gripperstate.state = [False, False, False, False, False, True]
            #rospy.loginfo(robot_name + "Gripper Closed")

        pubGripper.publish(gripperstate)
        rate.sleep()

def callback_gripper(data):
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