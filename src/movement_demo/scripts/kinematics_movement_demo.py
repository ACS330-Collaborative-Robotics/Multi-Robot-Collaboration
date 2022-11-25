#!/usr/bin/env python

# Name: Randomly select a block and publish ModelState message on /command_pos for each robot
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from gazebo_msgs.srv import GetModelState
from random import randint
from gazebo_msgs.msg import ModelState

def talker():
    # Define robot namespaces being used - also defines number of robots
    robot_namespaces = ["mover6_a", "mover6_b"]

    # Setup publisher on topic /command_pos used for both bots
    pub = rospy.Publisher("/command_pos", ModelState, queue_size=2)

    # Initialise ROS node
    rospy.init_node('joint_movement_demo')

    # Set time period
    T_period = 2
    rate = rospy.Rate(1/T_period)
    
    # Set fixed rotation
    a = 0
    b = 3.14 # End effector points downwards
    c = 0

    while not rospy.is_shutdown():
        # Alternate between the two robots
        for robot_num in range(len(robot_namespaces)):
            # Select random block
            block = "block" + str(randint(0,19))

            # Fetch block position relative to chosen robot
            xyz_pos = specific_block_pos(block, robot_namespaces[robot_num])

            # Initialise and fill ArmPos object
            arm_pos = ModelState()
            arm_pos.model_name = robot_namespaces[robot_num]

            arm_pos.pose.position.x = xyz_pos[0]
            arm_pos.pose.position.y = xyz_pos[1]
            arm_pos.pose.position.z = xyz_pos[2] + 0.05

            arm_pos.pose.orientation.x = a
            arm_pos.pose.orientation.y = b
            arm_pos.pose.orientation.z = c

            # Publish and log ArmPos
            pub.publish(arm_pos)
            rospy.loginfo(robot_namespaces[robot_num] + " " + block)

            # Sleep until time period has passed
            rate.sleep()

def specific_block_pos(specific_model_name, reference_model_name):
    # Use service to get position of specific block named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    data = model_state_service(specific_model_name, reference_model_name).pose.position

    # Return ModelState object with position relative to world 
    return [data.x, data.y, data.z]

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass