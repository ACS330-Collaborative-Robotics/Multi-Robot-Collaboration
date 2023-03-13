#!/usr/bin/env python

# Name: Gripper Controller
# Author: Tom Richards (tmrichards1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from cpr_robot.msg import ChannelStates
from sys import argv

global activateGripper
activateGripper = None

def main():
    rospy.init_node('gripper_controller')

    enableGripper = True

    global robot_name
    robot_name =  "/" + argv[1]
    print("----------------------------------")
    print(robot_name + " Gripper Initialised.")
    print("----------------------------------")
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pubGripper = rospy.Publisher(robot_name + '_p/OutputChannels', ChannelStates, queue_size=10)
        rospy.Subscriber(robot_name + "_p/gripper_state", Bool, callback_gripper)
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

def callback_gripper(data):
    global activateGripper
    activateGripper = data.data
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass