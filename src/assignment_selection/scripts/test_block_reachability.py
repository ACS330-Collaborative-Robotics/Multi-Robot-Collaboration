
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
from inv_kinematics.srv import InvKin, InvKinRequest
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
    block_orientation_euler = list(tf_conversions.transformations.euler_from_quaternion(block_orientation_quaternion))

    return is_block_position_reachable(pose.position.x, pose.position.y, pose.position.z, block_orientation_euler[0], block_orientation_euler[1], block_orientation_euler[2], robot_name, [0.09, 0.15])

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

        robots_can_reach = []
        for robot_name in robot_namespaces:
            if is_block_reachable(block_name, robot_name):
                block_names.append(block_name)
                robots_can_reach.append(robot_name)

        if len(robots_can_reach) == 0:
            rospy.logwarn("Assignment Selection - Ignoring %s as it is unreachable.", block_name)
        else:
            rospy.loginfo("Assignment Selection - Adding %s as it is reachable by %s.", block_name, ', '.join(robots_can_reach))
            
    rospy.loginfo("Assignment Selection - Block list built.\n")

    return block_names

def specific_block_pose(specific_model_name, reference_model_name) -> Pose:
    # Use service to get position of specific block named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    data = model_state_service(specific_model_name, reference_model_name).pose

    # Return ModelState object with position relative to world 
    return data

def is_block_position_reachable(x, y, z, euler_x, euler_y, euler_z, robot_name, z_offsets):
    rospy.wait_for_service('/inverse_kinematics_reachability')
    inv_kin_is_reachable = rospy.ServiceProxy('/inverse_kinematics_reachability', InvKin)
    
    inv_kin_request = InvKinRequest()

    inv_kin_request.state.pose.position.x = x
    inv_kin_request.state.pose.position.y = y
    inv_kin_request.state.pose.position.z = z

    orientation_euler = [0, math.pi, euler_z]
    orientation_quaternion = tf_conversions.transformations.quaternion_from_euler(orientation_euler[0], orientation_euler[1], orientation_euler[2])
    
    inv_kin_request.state.pose.orientation.x = orientation_quaternion[0]
    inv_kin_request.state.pose.orientation.y = orientation_quaternion[1]
    inv_kin_request.state.pose.orientation.z = orientation_quaternion[2]
    inv_kin_request.state.pose.orientation.w = orientation_quaternion[3]

    inv_kin_request.state.pose = frameConverter(robot_name, "world", inv_kin_request.state.pose)

    inv_kin_request.precise_orientation = True

    converted_z_height = inv_kin_request.state.pose.position.z

    for z_offset in z_offsets:
        inv_kin_request.state.pose.position.z = converted_z_height + z_offset
        if not inv_kin_is_reachable(inv_kin_request).success:
            break
    else:
        return True

    return False
    
def frameConverter(target_frame:str, reference_frame:str, goal_pose:Pose) -> Pose:
    # Setup tf2
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Setup time stamped pose object
    start_pose = PoseStamped()
    start_pose.pose = goal_pose

    start_pose.header.frame_id = reference_frame
    start_pose.header.stamp = rospy.get_rostime()

    # Convert from world frame to robot frame using tf2
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            new_pose = tfBuffer.transform(start_pose, target_frame+"/base_link")
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Error - Frame converter in Assignment Selection block_selection.py failed. Retrying now.")
            rate.sleep()
            continue
    
    return new_pose.pose


if __name__ == '__main__':
    try:
        block_testing()
    except rospy.ROSInterruptException:
        pass
