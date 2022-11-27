#!/usr/bin/env python

# Name: Randomly select a block and publish ModelState message on /command_pos for each robot
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from gazebo_msgs.srv import GetModelState
from random import randint
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String

def main():
    # Define robot namespaces being used - also defines number of robots
    robot_namespaces = ["mover6_a", "mover6_b"]

    # Setup publisher on topic /command_pos used for both bots
    pub = rospy.Publisher("/command_pos", ModelState, queue_size=2)

    # Setup subscriber callbacks 
    rospy.Subscriber("/mover6_a/next_block", String, mover6_a_callback)
    rospy.Subscriber("/mover6_b/next_block", String, mover6_b_callback)

    # Initialise ROS node
    rospy.init_node('kinematic_movement')

    rospy.spin()

def talker(robot_name_space, block_name):
    # Setup inverse_kinematics service
    rospy.wait_for_service('inverse_kinematics')
    inv_kin = rospy.ServiceProxy('inverse_kinematics', ModelState)

    print("Service setup.")

    # Set fixed rotation
    a = 0
    b = 3.14 # End effector points downwards
    c = 0

    # Select random block
    block = "block" + str(randint(1,20))

    # Fetch block position relative to chosen robot
    xyz_pos = specific_block_pos(block_name, robot_name_space)

    # Initialise and fill ArmPos object
    arm_pos = ModelState()
    arm_pos.model_name = robot_name_space

    arm_pos.pose.position.x = xyz_pos[0]
    arm_pos.pose.position.y = xyz_pos[1]
    arm_pos.pose.position.z = xyz_pos[2] + 0.05

    arm_pos.pose.orientation.x = a
    arm_pos.pose.orientation.y = b
    arm_pos.pose.orientation.z = c

    # Call inverse_kinematics service and log ArmPos
    inv_kin(arm_pos)
    rospy.loginfo(robot_name_space + " " + block)

def mover6_a_callback(data):
    talker("mover6_a", data.data)

def mover6_b_callback(data):
    talker("mover6_b", data.data)

def specific_block_pos(specific_model_name, reference_model_name):
    # Use service to get position of specific block named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    data = model_state_service(specific_model_name, reference_model_name).pose.position

    # Return ModelState object with position relative to world 
    return [data.x, data.y, data.z]

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass