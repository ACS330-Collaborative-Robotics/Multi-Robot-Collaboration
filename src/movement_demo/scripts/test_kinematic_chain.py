import rospy
import actionlib
import tf

from path_planning.msg import PathPlanAction, PathPlanGoal
from geometry_msgs.msg import Pose

def choose_block():
    goal = PathPlanGoal()

    #############################
    ## Configurable Parameters ##
    #############################

    goal.robot_name = "mover6_a"
    goal.block_name = "block18"

    cartesian_coordinates = [0.2, 0, 0.2]
    orientation_in_euler = [0, 0, 0]

    #############################

    # Declare ROS Node name
    rospy.init_node('test_kinematic_chain')

    # Setup path_planner action client
    path_client = actionlib.SimpleActionClient('path_planner', PathPlanAction)
    path_client.wait_for_server()

    end_pos = Pose()
    
    end_pos.position.x = cartesian_coordinates[0]
    end_pos.position.y = cartesian_coordinates[1]
    end_pos.position.z = cartesian_coordinates[2]

    quat = tf.transformations.quaternion_from_euler(orientation_in_euler[0],orientation_in_euler[1],orientation_in_euler[2])

    end_pos.orientation.x = quat[0]
    end_pos.orientation.y = quat[1]
    end_pos.orientation.z = quat[2]
    end_pos.orientation.w = quat[3]

    goal.end_pos = end_pos
    
    rospy.loginfo("Test Kinematic Chain - Calling path planner")
    
    path_client.send_goal(goal)

    while (path_client.get_state() == 1) and not rospy.is_shutdown():
        rospy.loginfo_once("Assignment Selection - Waiting for robot %s to complete action.", goal.robot_name)
        rospy.sleep(0.01)

    status = path_client.get_result().success
    if status:
        rospy.loginfo("Assignment Selection - Robot %s action completed successfully.\n", goal.robot_name)
    else:
        rospy.logerr("Assignment Selection - Robot %s action failed with status %i.\n", goal.robot_name, status)

if __name__ == '__main__':
    try:
        choose_block()
    except rospy.ROSInterruptException:
        pass
