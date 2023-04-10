# Name: Instruct robot joint to move then record and plot response
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from custom_msgs.msg import Joints
from sensor_msgs.msg import JointState

from math import pi
import matplotlib.pyplot as plt

simulation_state = []

def simulation_state_callback(data):
    joint_positions = list(data.position)
    simulation_state.append(joint_positions[joint_number-1])
    
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
    final_angle_degrees = 90
    
    time_delay_seconds = 5

    #############################

    initial_angle = initial_angle_degrees * pi/180
    final_angle = final_angle_degrees * pi/180
    
    positions_publisher = rospy.Publisher(robot_name + "/joint_angles", Joints, queue_size=10)
    joint_positions = [0 for i in range(6)]

    ## Initialise listeners
    simulation_positions_subscriber = rospy.Subscriber(robot_name + "/joint_states", JointState, simulation_state_callback)

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
    simulation_positions_subscriber.unregister()

    rospy.sleep(0.1) # Small delay for subscriber to unregister

    ## Plot response
    sample_number = range(0, len(simulation_state))
    plt.plot(sample_number, simulation_state)
    plt.show()

    rospy.logwarn("Plot displayed. Close plot to terminate script.")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass