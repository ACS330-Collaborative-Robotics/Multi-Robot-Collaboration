# Name: Constantly output joint error
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from custom_msgs.msg import Joints
from sensor_msgs.msg import JointState

from math import pi
import numpy as np

command_state = []
simulation_state = []

def command_state_callback(data):
    joint_positions = list(data.joints)

    global command_state
    command_state = joint_positions

def simulation_state_callback(data):
    joint_positions = list(data.position)

    global simulation_state
    simulation_state = joint_positions
    
def talker():
    rospy.init_node('joint_position_debug')

    robot_name = "mover6_a"

    ## Initialise listeners
    command_positions_subscriber = rospy.Subscriber(robot_name + "/joint_angles", Joints, command_state_callback)
    simulation_positions_subscriber = rospy.Subscriber(robot_name + "/joint_states", JointState, simulation_state_callback)

    rospy.sleep(0.1) # Small delay for publishers & subscribers to register

    while not rospy.is_shutdown():
        #print(command_state)
        #print(simulation_state)

        if len(command_state) == 6:
            joint_state_error_rad = np.array(command_state) - np.array(simulation_state[0:6])
            joint_state_error_degree = joint_state_error_rad * 180/pi
            rospy.loginfo("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n\n", joint_state_error_degree[0], joint_state_error_degree[1], joint_state_error_degree[2], joint_state_error_degree[3], joint_state_error_degree[4], joint_state_error_degree[5])

        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass