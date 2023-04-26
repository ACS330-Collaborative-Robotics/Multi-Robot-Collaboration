#!/usr/bin/env python

# Name: block_slection
# Author: Tom Richards (tomtommrichards@gmail.com), Conor Nichols (cjnichols1@sheffield.ac.uk), Annanthavel Santhanavel Ramesh(asanthanavelramesh1@sheffield.ac.uk)

import rospy
import actionlib
import tf
import tf2_ros
import tf_conversions


import numpy as np
import math
from operator import itemgetter

from gazebo_msgs.srv import GetModelState
from block_controller.msg import Blocks
from geometry_msgs.msg import Pose
from tf2_geometry_msgs import PoseStamped
from gazebo_msgs.msg import ModelState
from inv_kinematics.srv import InvKin
from path_planning.msg import PathPlanAction, PathPlanGoal

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

    # Setup path_planner action client
    path_clients = []
    for robot_name in robot_namespaces:
        path_clients.append(actionlib.SimpleActionClient(robot_name+'/path_planner', PathPlanAction))
        path_clients[-1].wait_for_server()

    robot_base_coords = getRobotBaseCoordinates(robot_namespaces)
    tower_origin_coordinates = [0, 0.3, 0]

    # Set Loop rate
    T = 5
    rate = rospy.Rate(1/T)

    ## Making array of block names ##

    # Wait for blockData to read in by subscriber
    while (blockData is None) and not(rospy.is_shutdown()):
        rospy.loginfo_once("Assignment Selection - Waiting for data.")
        rate.sleep()
    rospy.loginfo("Assignment Selection - Got block data.")

    # Iterate through blockData and retrieve list of block names
    blockNames = []
    for block_num in range(len(blockData.block_data)):
        block_name = "block" + str(blockData.block_data[block_num].block_number)

        for robot_name in robot_namespaces:
            if is_block_reachable(block_name, robot_name):
                blockNames.append(block_name)
                break
        else:
            rospy.logwarn("Assignment Selection - Ignoring %s as it is unreachable by %s.", block_name, robot_name)

    rospy.loginfo("Assignment Selection - Block list built.\n")

    # Getting distance from each robot to blocks and sellecting the smallest
    roboColect = []
    for blockName in blockNames:
        reldist = []
        for robot in robot_namespaces:
            temp_pose =  specific_block_pose(blockName, robot)
            temp = [temp_pose.position.x, temp_pose.position.y, temp_pose.position.z]
            reldist.append(math.sqrt(temp[0]**2+temp[1]**2+temp[2]**2))
        roboColect.append([blockName, reldist.index(min(reldist)), min(reldist)])
    
    # Ordering list on nearest
    sorted(roboColect,key=itemgetter(2))
    
    # Splitting into seperate lists
    goCollect = []
    for i in range(len(robot_namespaces)):
        goCollect.append([])
        for nextBlock in roboColect:
            if nextBlock[1] == i:
                goCollect[i].append(nextBlock[0])

    # Setup tower block locations
    number_layers = math.floor(len(blockNames)/2) #num of layers
    tower_block_positions = [] #this has to be a 3 column * layers(value) matrix
    height=0 #height of blocks

    euler_a = 0
    euler_b = 0
    euler_c = 0

    # Generate coordinates
    for i in range(number_layers):
        width = 0 #width of blocks
        home_pos = [width, 0, height, euler_a, euler_b, euler_c]

        for j in range(2):
            home_pos = [width, 0, height, euler_a, euler_b, euler_c]
            tower_block_positions.append(home_pos)
            width = width + 0.08
        height = height + 0.04

        if euler_c == 0:
            euler_c = -90*(math.pi/180)
        elif euler_c == -90*(math.pi/180):
            euler_c = 0

    rospy.loginfo("Assignment Selection - Assignment goal list complete. Beginnning publishing.")

    robots_cannot_reach_next_block = [False for x in range(len(robot_namespaces))]
    robots_busy = [False for x in range(len(robot_namespaces))]

    # Publish assignments
    while len(tower_block_positions) > 0 and len(blockNames) > 0 and not rospy.is_shutdown():
        # Check if each robot is busy
        for robot_number_iterator in range(len(robot_namespaces)):
            # actionlib states: https://get-help.robotigniteacademy.com/t/get-state-responses-are-incorrect/6680
            robots_busy[robot_number_iterator] = not (path_clients[robot_number_iterator].get_state() in [0, 3, 4, 9]) 
            #TODO: Add error handling 

        unavailable_robots = list(np.logical_or(robots_cannot_reach_next_block, robots_busy))

        # IF a robot is availible, attempt to allocate a task
        if not all(unavailable_robots):
            robot_number = unavailable_robots.index(False)

            if not allocate_task(str(blockNames[0]), str(robot_namespaces[robot_number]), robot_number, tower_block_positions, tower_origin_coordinates, path_clients):
                robots_cannot_reach_next_block[robot_number] = True
            else:
                robots_cannot_reach_next_block = [False for x in range(len(robot_namespaces))]
                blockNames.pop(0)

        elif all(robots_busy):
            rospy.logwarn("Assignment Selection - All robots busy, waiting till one is free.")
            rospy.sleep(0.5)

        elif all(robots_cannot_reach_next_block):
            rospy.logerr("Assignment Selection - No robots available for %s. Skipping block.", str(blockNames[0]))
            blockNames.pop(0)
            robots_cannot_reach_next_block = [False for x in range(len(robot_namespaces))]
            
        rospy.sleep(0.1)

'''while (path_clients[robot_number].get_state() == 1) and not rospy.is_shutdown():
    rospy.loginfo_once("Assignment Selection - Waiting for robot %s to complete action.", goal.robot_name)
    rospy.sleep(0.01)

status = path_clients[robot_number].get_result()
if status == None:
    rospy.logfatal("\n\nAssignment Selection - Path Client returned None. Investigate Source.\n\n")
elif status.success:
    rospy.loginfo("Assignment Selection - Robot %s action completed successfully.\n", goal.robot_name)
else:
    rospy.logerr("Assignment Selection - Robot %s action failed with status %i.\n", goal.robot_name, status.success)'''  

def allocate_task(block_name, robot_name, robot_number, tower_block_positions, tower_origin_coordinates, path_clients) -> bool:
    if not is_block_reachable(block_name, robot_name):
        rospy.logwarn("Assignment Selection - Cannot Allocate %s to %s.", block_name, robot_name)
        return False
    else:
        rospy.loginfo("Assignment Selection - Allocating %s to %s.", block_name, robot_name)

        goal = PathPlanGoal()
        goal.block_name = block_name
        goal.robot_name = robot_name

        end_pos = Pose()
        end_pos.position.x = tower_block_positions[0][0] + tower_origin_coordinates[0]
        end_pos.position.y = tower_block_positions[0][1] + tower_origin_coordinates[1]
        end_pos.position.z = tower_block_positions[0][2] + tower_origin_coordinates[2]

        quat = tf.transformations.quaternion_from_euler(
                tower_block_positions[0][3],tower_block_positions[0][4],tower_block_positions[0][5])
        end_pos.orientation.x = quat[0]
        end_pos.orientation.y = quat[1]
        end_pos.orientation.z = quat[2]
        end_pos.orientation.w = quat[3]

        if not is_block_position_reachable(end_pos.position.x, end_pos.position.y, end_pos.position.z, tower_block_positions[0][3],tower_block_positions[0][4],tower_block_positions[0][5], robot_name):
            rospy.logwarn("Assignment Selection - Cannot reach final block position with %s.", robot_name)
            return False

        goal.end_pos = end_pos

        path_clients[robot_number].send_goal(goal)
        tower_block_positions.pop(0)

        rospy.sleep(0.01)

        return True

def specific_block_pose(specific_model_name, reference_model_name) -> Pose:
    # Use service to get position of specific block named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    data = model_state_service(specific_model_name, reference_model_name).pose

    # Return ModelState object with position relative to world 
    return data

def is_block_reachable(block_name, robot_name) -> bool:
    pose = specific_block_pose(block_name, "world")

    block_orientation_quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    block_orientation_euler = tf_conversions.transformations.euler_from_quaternion(block_orientation_quaternion)

    return is_block_position_reachable(pose.position.x, pose.position.y, pose.position.z, block_orientation_euler[0], block_orientation_euler[1], block_orientation_euler[2], robot_name)

def is_block_position_reachable(x, y, z, euler_x, euler_y, euler_z, robot_name):
    rospy.wait_for_service('inverse_kinematics_reachability')
    inv_kin_is_reachable = rospy.ServiceProxy('inverse_kinematics_reachability', InvKin)

    model_state = ModelState()

    model_state.pose.position.x = x
    model_state.pose.position.y = y
    model_state.pose.position.z = z

    block_orientation_euler = [euler_x, euler_y, euler_z]

    orientation_euler = [0, math.pi, block_orientation_euler[2]]
    orientation_quaternion = tf_conversions.transformations.quaternion_from_euler(orientation_euler[0], orientation_euler[1], orientation_euler[2])
    
    model_state.pose.orientation.x = orientation_quaternion[0]
    model_state.pose.orientation.y = orientation_quaternion[1]
    model_state.pose.orientation.z = orientation_quaternion[2]
    model_state.pose.orientation.w = orientation_quaternion[3]

    model_state.pose = frameConverter(robot_name, "world", model_state.pose)

    # Test at two heights above the block
    model_state.pose.position.z += 0.10
    if inv_kin_is_reachable(model_state).success:

        model_state.pose.position.z += 0.05
        if inv_kin_is_reachable(model_state).success:
            #rospy.loginfo("Assignment Selection - Adding %s as it is reachable by %s", block_name, robot_name)
            return True
    
    #rospy.loginfo("Assignment Selection - %s cannot reach %s", robot_name, block_name)
    return False
    

def getRobotBaseCoordinates(robot_namespaces):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    base_coordinates = []
    for robot_name in robot_namespaces:
        robot_base_coordinates = []
        while not tfBuffer.can_transform("world", robot_name+"_base", rospy.Time(0)) and not rospy.is_shutdown():
            rospy.logwarn("Cannot find robot base transform - block_selection.py. Retrying now.")
            rospy.sleep(0.1)
        
        transform_response = tfBuffer.lookup_transform("world", robot_name+"_base", rospy.Time(0))

        robot_base_coordinates.append(transform_response.transform.translation.x)
        robot_base_coordinates.append(transform_response.transform.translation.y)

        base_coordinates.append(robot_base_coordinates)

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
            rospy.logwarn("Error - Frame converter in Path Planner ServiceHelper.py failed. Retrying now.")
            rate.sleep()
            continue
    
    return new_pose.pose

if __name__ == '__main__':
    try:
        assignment_selector()
    except rospy.ROSInterruptException:
        pass
