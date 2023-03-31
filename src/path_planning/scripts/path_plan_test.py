# Name: path planner test 
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
import actionlib

from path_planning.msg import PathPlanAction, PathPlanGoal

def path_plan_test():
    # Declare ROS Node name
    rospy.init_node('path_plan_test')
    
    # Define robot namespace being used
    robot_name = "mover6_a"
    
    # Setup path_planner action client
    path_client = actionlib.SimpleActionClient('path_planner', PathPlanAction)
    path_client.wait_for_server()



if __name__ == '__main__':
    try:
        path_plan_test()
    except rospy.ROSInterruptException:
        pass
