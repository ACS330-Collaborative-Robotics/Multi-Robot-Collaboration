
#!/usr/bin/env python

# Name: test_block_reachability
# Author: Tom Richards (tomtommrichards@gmail.com)
import rospy
import actionlib
import tf
import tf2_ros
import tf_conversions

import numpy as np
import math

from gazebo_msgs.srv import GetModelState
from block_controller.msg import Blocks
from geometry_msgs.msg import Pose
from tf2_geometry_msgs import PoseStamped
from gazebo_msgs.msg import ModelState
from inv_kinematics.srv import InvKin
from path_planning.msg import PathPlanAction, PathPlanGoal
from block_controller.srv import UpdateBlocks
from custom_msgs.msg import Joints

# Global variable to store blockData as it appears from subscriber
blockData = None

def callback(data):
    global blockData
    blockData = data

def block_testing():
    # Declare ROS Node name
    rospy.init_node('block_selector')

    # Setup block_pos listener
    rospy.Subscriber('/blocks_pos', Blocks, callback)

    # Define robot namespaces being used - also defines number of robots
    robot_namespaces = ["mover6_a", "mover6_b"]

    update_block_positions()

    # Setup path_planner action client
    path_clients = []
    for robot_name in robot_namespaces:
        path_clients.append(actionlib.SimpleActionClient(robot_name+'/path_planner', PathPlanAction))
        path_clients[-1].wait_for_server()

    #############################
    ## Configurable Parameters ##
    #############################

    # tower_origin_coordinates = [x, y, z]
    tower_origin_coordinates = [0.1, 0.36, 0.02]

    use_manual_block_locations = False
    manual_block_location_xyz = [[0.1, -0.1, 0], [0.1, 0, 0], [0.1, 0.1, 0], [0.1, -0.1+0.72, 0], [0.1, 0+0.72, 0], [0.1, 0.1+0.72, 0]]
    manual_block_location_euler_rotation = [0, 0, 0]

    enable_home_between_assignments = True
    home_joint_positions = [90*math.pi/180, 0, 0, 0, 0, 0]

    block_width = 0.035
    block_height = 0.035
    block_length = 0.105

    maximum_simulatenous_robots = 1
    #maximum_simulatenous_robots = len(robot_namespaces)

    #############################

    joint_pubs = []
    if enable_home_between_assignments:
        for robot_name in robot_namespaces:
            joint_pubs.append(rospy.Publisher(robot_name + "/joint_angles", Joints, queue_size=10))
    
    ## Making array of block names
    block_names = build_block_list(robot_namespaces)

    rospy.sleep(0.1)
    
def update_block_positions():
    try:
        rospy.wait_for_service('block_update', 1)
    except rospy.ROSException:
        return False
    
    block_update = rospy.ServiceProxy('block_update', UpdateBlocks)

    try:
        block_update(True)
    except rospy.ServiceException:
        rospy.logwarn("Assignment Selection - Block update service call failed.")

def is_block_reachable(block_name, robot_name) -> bool:
    pose = specific_block_pose(block_name, "world")

    block_orientation_quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    block_orientation_euler = tf_conversions.transformations.euler_from_quaternion(block_orientation_quaternion)

    return is_block_position_reachable(pose.position.x, pose.position.y, pose.position.z, block_orientation_euler[0], block_orientation_euler[1], block_orientation_euler[2], robot_name)


def build_block_list(robot_namespaces):
    # Wait for blockData to read in by subscriber
    while (blockData is None) and not(rospy.is_shutdown()):
        rospy.loginfo_once("Assignment Selection - Waiting for data.")
        rospy.sleep(0.2)
    rospy.loginfo("Assignment Selection - Got block data.")

    # Iterate through blockData and retrieve list of block names
    block_names = []
    for block_num in range(len(blockData.block_data)):
        block_name = "block" + str(blockData.block_data[block_num].block_number)

        for robot_name in robot_namespaces:
            if is_block_reachable(block_name, robot_name):
                block_names.append(block_name)
                rospy.logwarn("Assignment Selection - Adding %s as it is reachable.", block_name)
                break
        else:
            rospy.logwarn("Assignment Selection - Ignoring %s as it is unreachable.", block_name)

    rospy.loginfo("Assignment Selection - Block list built.\n")

    return block_names


if __name__ == '__main__':
    try:
        block_testing()
    except rospy.ROSInterruptException:
        pass
