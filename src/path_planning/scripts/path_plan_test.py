# Name: path planner test 
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
import actionlib

from path_planning.msg import PathPlanAction, PathPlanGoal

from math import pi

def path_plan_test():
    ###############################
    ##  Configurable parameters  ##
    ###############################

    # Initial Robot Cartesian Position
    initial_robot_position_x = 0
    initial_robot_position_y = 0.2
    initial_robot_position_z = 0.2

    # Initial Robot Euler Angle Orientation
    initial_robot_orientation_x = 0*pi/180
    initial_robot_orientation_y = 180*pi/180 # 180 degrees = end effector point down
    initial_robot_orientation_z = 0*pi/180

    # Final Robot Cartesian Position
    final_robot_position_x = 0.1
    final_robot_position_y = 0.3
    final_robot_position_z = 0.2

    # Final Robot Euler Angle Orientation
    final_robot_orientation_x = 0*pi/180
    final_robot_orientation_y = 180*pi/180
    final_robot_orientation_z = 0*pi/180

    # Block Cartesian Position
    initial_robot_position_x = 0.1
    initial_robot_position_y = 0.1
    initial_robot_position_z = 0.02

    # Block Euler Angle Orientation
    initial_robot_orientation_x = 0*pi/180
    initial_robot_orientation_y = 0*pi/180
    initial_robot_orientation_z = 45*pi/180

    ######################################
    ##  End of Configurable parameters  ##
    ######################################

    # Declare ROS Node name
    rospy.init_node('path_plan_test')
    
    # Define robot namespace being used
    robot_name = "mover6_a"
    
    # Setup path_planner action client
    path_client = actionlib.SimpleActionClient('path_planner', PathPlanAction)
    path_client.wait_for_server()

    #TODO Spawn Block

    #TODO Set Initial Pose via IK

    #TODO Call Path Planner

    #TODO Cleanup Block
    

if __name__ == '__main__':
    try:
        path_plan_test()
    except rospy.ROSInterruptException:
        pass
