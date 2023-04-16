# Name: path planner test 
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
import actionlib
from pathlib import Path
import tf

from path_planning.msg import PathPlanAction, PathPlanGoal
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, DeleteModel
from inv_kinematics.srv import InvKin

from math import pi

def path_plan_test():
    ###############################
    ##  Configurable parameters  ##
    ###############################

    # Initial Robot Cartesian Position
    initial_robot_position_x = 0
    initial_robot_position_y = 0.25
    initial_robot_position_z = 0.20

    # Initial Robot Euler Angle Orientation
    initial_robot_orientation_x = 0*pi/180
    initial_robot_orientation_y = 180*pi/180 # 180 degrees = end effector point down
    initial_robot_orientation_z = 0*pi/180

    # Final Robot Cartesian Position
    final_robot_position_x = 0
    final_robot_position_y = 0
    final_robot_position_z = 0

    # Final Robot Euler Angle Orientation
    final_robot_orientation_x = 0*pi/180
    final_robot_orientation_y = 180*pi/180
    final_robot_orientation_z = 0*pi/180

    # Block Cartesian Position
    block_robot_position_x = 0.18
    block_robot_position_y = 0.20
    block_robot_position_z = 0.05

    # Block Euler Angle Orientation
    block_robot_orientation_x = 0*pi/180
    block_robot_orientation_y = 0*pi/180
    block_robot_orientation_z = 90*pi/180

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

    rospy.wait_for_service('inverse_kinematics')
    inv_kin_srv = rospy.ServiceProxy('inverse_kinematics', InvKin)

    ## Spawn Block ##
    block_name = "block0"

    # Open Block URDF
    f = open(str(Path.home()) + '/catkin_ws/src/block_controller/urdf/block.urdf')
    urdf = f.read()

    # Create Block Pose object
    block_pose = Pose()
    block_pose.position.x = block_robot_position_x
    block_pose.position.y = block_robot_position_y
    block_pose.position.z = block_robot_position_z

    quat = tf.transformations.quaternion_from_euler(block_robot_orientation_x, block_robot_orientation_y, block_robot_orientation_z)
    block_pose.orientation.x = quat[0]
    block_pose.orientation.y = quat[1]
    block_pose.orientation.z = quat[2]
    block_pose.orientation.w = quat[3]

    # Setup URDF spawner service
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    urdf_spawner = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)

    rospy.logwarn(urdf_spawner(block_name, urdf, "blocks", block_pose, "world").status_message)

    rospy.sleep(2)

    ## Set Initial Pose via IK ##

    # Create Initial Pose object
    initial_pose = Pose()
    initial_pose.position.x = initial_robot_position_x
    initial_pose.position.y = initial_robot_position_y
    initial_pose.position.z = initial_robot_position_z

    quat = tf.transformations.quaternion_from_euler(initial_robot_orientation_x, initial_robot_orientation_y, initial_robot_orientation_z)
    initial_pose.orientation.x = quat[0]
    initial_pose.orientation.y = quat[1]
    initial_pose.orientation.z = quat[2]
    initial_pose.orientation.w = quat[3]

    # Initialise and fill ModelState object for ik
    initial_arm_state = ModelState()
    initial_arm_state.model_name = robot_name

    initial_arm_state.pose = initial_pose
    # Call inverse_kinematics service
    status = inv_kin_srv(initial_arm_state,0).success

    if status:
        rospy.logwarn("Robot Initial Pose set succesfully.")
    else:
        rospy.logfatal("Robot Initial Pose failed to set.")

    rospy.sleep(2)

    ## Call Path Planner ##

    goal = PathPlanGoal()

    goal.block_name = block_name
    goal.robot_name = robot_name

    final_pose = Pose()
    final_pose.position.x = final_robot_position_x
    final_pose.position.y = final_robot_position_y
    final_pose.position.z = final_robot_position_z

    quat = tf.transformations.quaternion_from_euler(final_robot_orientation_x, final_robot_orientation_y, final_robot_orientation_z)
    final_pose.orientation.x = quat[0]
    final_pose.orientation.y = quat[1]
    final_pose.orientation.z = quat[2]
    final_pose.orientation.w = quat[3]

    goal.end_pos = final_pose

    path_client.send_goal(goal)

    rospy.sleep(0.01)

    while (path_client.get_state() == 1) and not rospy.is_shutdown():
        rospy.loginfo_once("Assignment Selection - Waiting for robot %s to complete action.", goal.robot_name)
        rospy.sleep(0.01)

    status = path_client.get_result().success
    if status:
        rospy.logwarn("Assignment Selection - Robot %s action completed successfully.\n", goal.robot_name)
    else:
        rospy.logerr("Assignment Selection - Robot %s action failed with status %i.\n", goal.robot_name, status)

    rospy.sleep(2)

    ## Cleanup Block ##

    # Setup model removal service
    rospy.wait_for_service('gazebo/delete_model')
    model_remover = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

    rospy.logwarn(model_remover(block_name).status_message)


if __name__ == '__main__':
    try:
        path_plan_test()
    except rospy.ROSInterruptException:
        pass
