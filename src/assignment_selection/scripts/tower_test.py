#!/usr/bin/env python

# Name: block_slection
# Author: Tom Richards (tomtommrichards@gmail.com), Conor Nichols (cjnichols1@sheffield.ac.uk), Annanthavel Santhanavel Ramesh(asanthanavelramesh1@sheffield.ac.uk)

import rospy
import tf2_ros
import tf_conversions

import numpy as np
import math

from gazebo_msgs.srv import GetModelState
from block_controller.msg import Blocks
from geometry_msgs.msg import Pose
from tf2_geometry_msgs import PoseStamped
from inv_kinematics.srv import InvKin, InvKinRequest
from block_controller.srv import UpdateBlocks

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Global variable to store blockData as it appears from subscriber
blockData = None

def callback(data):
    global blockData
    blockData = data

def assignment_selector():
    # Declare ROS Node name
    rospy.init_node('block_selector')

    # Setup block_pos listener
    rospy.Subscriber('/blocks_pos', Blocks, callback)

    # Define robot namespaces being used - also defines number of robots
    robot_namespaces = ["mover6_a", "mover6_b"]

    #############################
    ## Configurable Parameters ##
    #############################

    # tower_origin_coordinates = [x, y, z]
    tower_origin_coordinates = [0, 0.365, 0.02]

    block_width = 0.035
    block_height = 0.035
    block_length = 0.105

    #############################
    
    ## Making array of block names
    block_names = build_block_list(robot_namespaces, True)
    print()

    tower_block_positions = generate_tower_block_positions(len(block_names), block_width, block_height, block_length)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    for tower_block_position in tower_block_positions:
        print(tower_block_position)
        for robot_name in robot_namespaces:
            x = tower_block_position[0] + tower_origin_coordinates[0]
            y = tower_block_position[1] + tower_origin_coordinates[1]
            z = tower_block_position[2] + tower_origin_coordinates[2]

            ax.scatter(x, y, z, c='k')

            # Ensure end position is reachable
            if not is_block_position_reachable(x, y, z, tower_block_position[3],tower_block_position[4],tower_block_position[5], robot_name, [0.1, 0.2]):
                rospy.logwarn("Assignment Selection - Cannot reach final block position with %s.", robot_name)

    plt.show()

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

def generate_tower_block_positions(number_of_blocks, block_width, block_height, block_length):
    tower_block_positions = []
    number_of_blocks_per_circle = 8
    circle_radius = 0.15

    x_initial = circle_radius
    y_initial = 0

    z = 0
    euler_x = 0
    euler_y = 0
    euler_z = 0
    for block_angle_multiplier in range(number_of_blocks_per_circle):
        block_angle = block_angle_multiplier * 2 * math.pi / number_of_blocks_per_circle

        x = x_initial*math.cos(block_angle) - y_initial*math.sin(block_angle)
        y = x_initial*math.sin(block_angle) + y_initial*math.cos(block_angle)

        tower_block_positions.append([x, y, z, euler_x, euler_y, block_angle+math.pi/2])
        
    return tower_block_positions

def generate_tower_block_positions_old(number_of_blocks, block_width, block_height, block_length):
    # Setup tower block locations
    number_layers = math.floor(number_of_blocks/4) #num of layers
    tower_block_positions = [] #this has to be a 3 column * layers(value) matrix
    height = 0 #height of blocks

    # Generate coordinates
    for i in range(number_layers):
        width = 0 #width of blocks
        home_pos = [width, 0, height, 0, 0, math.pi/2]

        for j in range(4):
            home_pos = [width, 0, height, 0, 0, math.pi/2]
            tower_block_positions.append(home_pos)
            width = width + 2*block_width

        height = height + block_height
    
    return tower_block_positions

def build_block_list(robot_namespaces, debug=False):
    # Wait for blockData to read in by subscriber
    while (blockData is None) and not(rospy.is_shutdown()):
        rospy.loginfo_once("Assignment Selection - Waiting for data.")
        rospy.sleep(0.2)
    #rospy.loginfo("Assignment Selection - Got block data.")

    # Iterate through blockData and retrieve list of block names
    block_names = []
    for block_num in range(len(blockData.block_data)):
        block_name = "block" + str(blockData.block_data[block_num].block_number)

        robots_can_reach = []
        for robot_name in robot_namespaces:
            if is_block_reachable(block_name, robot_name, [0.09, 0.15]):
                block_names.append(block_name)
                robots_can_reach.append(robot_name)

        if debug:
            if len(robots_can_reach) == 0:
                rospy.logwarn("Assignment Selection - Ignoring %s as it is unreachable.", block_name)
            else:
                rospy.loginfo("Assignment Selection - Adding %s as it is reachable by %s.", block_name, ', '.join(robots_can_reach))

    return block_names

def specific_block_pose(specific_model_name, reference_model_name) -> Pose:
    # Use service to get position of specific block named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    data = model_state_service(specific_model_name, reference_model_name).pose

    # Return ModelState object with position relative to world 
    return data

def is_block_reachable(block_name, robot_name, z_offsets) -> bool:
    pose = specific_block_pose(block_name, "world")

    block_orientation_quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    block_orientation_euler = tf_conversions.transformations.euler_from_quaternion(block_orientation_quaternion)

    return is_block_position_reachable(pose.position.x, pose.position.y, pose.position.z, block_orientation_euler[0], block_orientation_euler[1], block_orientation_euler[2], robot_name, z_offsets)

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
    
def getRobotBaseCoordinates(robot_namespaces):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    base_coordinates = []
    for robot_name in robot_namespaces:
        robot_base_coordinates = []
        while not tfBuffer.can_transform("world", robot_name+"/base_link", rospy.Time(0)) and not rospy.is_shutdown():
            rospy.loginfo("Cannot find robot base transform - block_selection.py. Retrying now.")
            rospy.sleep(0.05)
        
        transform_response = tfBuffer.lookup_transform("world", robot_name+"/base_link", rospy.Time(0))

        robot_base_coordinates.append(transform_response.transform.translation.x)
        robot_base_coordinates.append(transform_response.transform.translation.y)

        base_coordinates.append(robot_base_coordinates)
    
    return base_coordinates

def frameConverter(target_frame:str, reference_frame:str, goal_pose:Pose) -> Pose:
    # Setup tf2
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Setup time stamped pose object
    start_pose = PoseStamped()
    start_pose.pose = goal_pose

    orientation_in_quaternion = [start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w]
    orientation_in_euler = tf_conversions.transformations.euler_from_quaternion(orientation_in_quaternion)

    #rospy.loginfo("Frame Converter - Start pose:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f", start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z, orientation_in_euler[0]*180/math.pi, orientation_in_euler[1]*180/math.pi, orientation_in_euler[2]*180/math.pi)

    start_pose.header.frame_id = reference_frame
    start_pose.header.stamp = rospy.get_rostime()

    # Convert from world frame to robot frame using tf2
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            new_pose = tfBuffer.transform(start_pose, target_frame+"/base_link")
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Error - Frame converter in Path Planner ServiceHelper.py failed. Retrying now.")
            rate.sleep()
            continue
    
    orientation_in_quaternion = [new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w]
    orientation_in_euler = tf_conversions.transformations.euler_from_quaternion(orientation_in_quaternion)
    
    #rospy.loginfo("Frame Converter - New pose:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f", new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z, orientation_in_euler[0]*180/math.pi, orientation_in_euler[1]*180/math.pi, orientation_in_euler[2]*180/math.pi)

    return new_pose.pose

if __name__ == '__main__':
    try:
        assignment_selector()
    except rospy.ROSInterruptException:
        pass
