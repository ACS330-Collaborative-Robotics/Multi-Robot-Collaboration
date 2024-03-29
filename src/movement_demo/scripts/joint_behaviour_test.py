# Name: Instruct robot joint to move then record and plot response
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from custom_msgs.msg import Joints
from sensor_msgs.msg import JointState

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
    
def talker():
    rospy.init_node('joint_behaviour_test')

    global joint_number

    #############################
    ## Configurable Parameters ##
    #############################

    robot_name = "mover6_a"
    joint_number = 1 # Range: 1-6

    # Initial Angle -> Time Delay -> Final Angle -> Time Delay
    initial_angle_degrees = 0
    final_angle_degrees = 0
    
    time_delay_seconds = 16

    #############################

    initial_angle = initial_angle_degrees * pi/180
    final_angle = final_angle_degrees * pi/180
    
    positions_publisher = rospy.Publisher(robot_name + "/joint_angles", Joints, queue_size=10)
    joint_positions = [-pi/2,0,0,0,0,0]

    ## Initialise listeners
    command_positions_subscriber = rospy.Subscriber(robot_name + "/joint_angles", Joints, command_state_callback)
    simulation_positions_subscriber = rospy.Subscriber(robot_name + "/joint_states", JointState, simulation_state_callback)
    physical_positions_subscriber = rospy.Subscriber(robot_name + "_p/joint_states", JointState, physical_state_callback)

    rospy.sleep(0.1) # Small delay for publishers & subscribers to register

    ## Publish Initial Angle
    joint_positions[joint_number-1] = initial_angle
    positions_publisher.publish(joint_positions)

    rospy.logwarn("Publishing joint %d to angle %.2f", joint_number, initial_angle_degrees)
    rospy.sleep(time_delay_seconds)

    ## Publish Final Angle
    joint_positions[joint_number-1] = final_angle
    positions_publisher.publish(joint_positions)

    rospy.logwarn("Publishing joint %d to angle %.2f", joint_number, final_angle_degrees)
    rospy.sleep(time_delay_seconds)

    ## Disable subscribers
    command_positions_subscriber.unregister()
    simulation_positions_subscriber.unregister()
    physical_positions_subscriber.unregister()

    rospy.sleep(0.1) # Small delay for subscriber to unregister

    ## Plot response
    zero_time = command_state[0][0]
    command_time_array = np.array(command_state[0]) - zero_time

    plt.scatter(command_time_array, command_state[1], color="red")

    command_line_time = np.linspace(0, time_delay_seconds*2, 20)
    command_line_initial = np.full(command_line_time.shape, initial_angle)
    command_line_final = np.full(command_line_time.shape, final_angle)

    plt.plot(command_line_time, command_line_initial, "r:", label="Command")
    plt.plot(command_line_time, command_line_final, "r:")

    simulation_time_array = np.array(simulation_state[0]) - zero_time

    plt.plot(simulation_time_array, simulation_state[1], "k:*", label="Simulation")

    physical_time_array = np.array(physical_state_time) - zero_time

    plt.plot(physical_time_array, physical_state_state, "b:*", label="Physical")

    plt.xlabel("Time (s)")
    plt.ylabel("Joint Angle (radians)")
    plt.title("Joint " + str(joint_number))
    plt.legend(loc=7)

    rospy.logwarn("Plot displaying. Close plot to terminate script.")
    plt.show()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass