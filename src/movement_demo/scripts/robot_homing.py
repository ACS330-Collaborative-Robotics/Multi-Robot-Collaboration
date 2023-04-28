# Name: Instruct robot joint to move then record and plot response
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from custom_msgs.msg import Joints
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from math import pi
import matplotlib.pyplot as plt
import numpy as np

command_state = [[],[]]
simulation_state = [[],[]]
physical_state_time = []
physical_state_state = []

def command_state_callback(data):
    joint_positions = list(data.joints)
    time = rospy.get_time()

    command_state[0].append(time)
    command_state[1].append(joint_positions[joint_number-1])

def simulation_state_callback(data):
    joint_positions = list(data.position)
    time = data.header.stamp.to_sec()

    simulation_state[0].append(time)
    simulation_state[1].append(joint_positions[joint_number-1])

def physical_state_callback(data):
    joint_positions = list(data.position)
    time = rospy.get_time()
    if len(joint_positions) == 6:

        physical_state_time.append(time)
        physical_state_state.append(joint_positions[joint_number-1])
    
def talker(robot_name, angle, joint,GripperState):
    rospy.init_node('joint_behaviour_test')

    global joint_number
    joint_number = joint

    #############################
    ## Configurable Parameters ##
    #############################

    # Initial Angle -> Time Delay -> Final Angle -> Time Delay
    initial_angle_degrees = angle
    final_angle_degrees = angle
    
    time_delay_seconds = 8

    #############################

    initial_angle = initial_angle_degrees * pi/180
    final_angle = final_angle_degrees * pi/180
    
    positions_publisher = rospy.Publisher(robot_name + "/joint_angles", Joints, queue_size=10)
    joint_positions = [-pi/2,0,0,0,0,0]

    ## Initialise listeners
    command_positions_subscriber = rospy.Subscriber(robot_name + "/joint_angles", Joints, command_state_callback)
    simulation_positions_subscriber = rospy.Subscriber(robot_name + "/joint_states", JointState, simulation_state_callback)
    physical_positions_subscriber = rospy.Subscriber(robot_name + "_p/joint_states", JointState, physical_state_callback)
    pubGripper = rospy.Publisher(robot_name+'/gripper_state', Bool, queue_size=10)

    rospy.sleep(0.1) # Small delay for publishers & subscribers to register

    ## Publish Initial Angle
    joint_positions[joint_number-1] = initial_angle
    positions_publisher.publish(joint_positions)

    rospy.logwarn("Publishing joint %d to angle %.2f", joint_number, initial_angle_degrees)
    rospy.sleep(time_delay_seconds)


    ## opening the gripper
    rospy.logwarn("Opening Gipper")
    pubGripper.publish(GripperState)

    ## Disable subscribers
    command_positions_subscriber.unregister()
    simulation_positions_subscriber.unregister()
    physical_positions_subscriber.unregister()
    pubGripper.unregister()


    

    rospy.sleep(0.1) # Small delay for subscriber to unregister

if __name__ == '__main__':
    try:
        talker("mover6_b",90,1,True)
        talker("mover6_a",90,1,True)
        #talker("mover6_a",90,1)
        #talker("mover6_a",0,2)
    except rospy.ROSInterruptException:
        pass