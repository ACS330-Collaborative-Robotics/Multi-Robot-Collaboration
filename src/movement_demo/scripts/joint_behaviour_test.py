# Name: Instruct robot joint to move then record and plot response
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from custom_msgs.msg import Joints
from math import pi

def talker():
    rospy.init_node('joint_behaviour_test')

    #############################
    ## Configurable Parameters ##
    #############################

    robot_name = "mover6_a"
    joint_number = 1 # Range: 1-6

    # Initial Angle -> Time Delay -> Final Angle -> Time Delay
    initial_angle_degrees = 0
    final_angle_degrees = 90
    
    time_delay_seconds = 5

    #############################

    initial_angle = initial_angle_degrees * pi/180
    final_angle = final_angle_degrees * pi/180

    joint_position_pub = rospy.Publisher(robot_name + "/joint_angles", Joints, queue_size=10)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass