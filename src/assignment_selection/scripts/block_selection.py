#!/usr/bin/env python

# Name: block_slection
# Author: Tom Richards (tomtommrichards@gmail.com), Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy

from gazebo_msgs.srv import GetModelState
from block_controller.msg import Blocks
from path_planning.srv import PathPlan
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from inv_kinematics.srv import InvKin

import math
from operator import itemgetter
import tf_conversions

# Global variable to store blockData as it appears from subscriber
blockData = None

def callback(data):
    global blockData
    blockData = data

def choose_block():
    # Setup block_pos listener
    rospy.Subscriber('/blocks_pos', Blocks, callback)

    # Define robot namespaces being used - also defines number of robots
    robot_namespaces = ["mover6_a", "mover6_b"]
    
    # Setup path_planner service
    rospy.wait_for_service('path_planner')
    path_service = rospy.ServiceProxy('path_planner', PathPlan)

    # Declare ROS Node name
    rospy.init_node('block_selector')

    # Set Loop rate
    T = 5
    rate = rospy.Rate(1/T)

    while (blockData is None) and not(rospy.is_shutdown()):
        rospy.loginfo("Block Selection - Waiting for data.")
        rospy.sleep(0.1)
    rospy.loginfo("Block Selection - Got block data,")

    # Loop Selection until script is terminated
    while not rospy.is_shutdown():
        ## Making array of block names ##

        # Wait for blockData to read in by subscriber
        while (blockData is None) and not(rospy.is_shutdown()):
            rospy.loginfo("Block Selection - Waiting for data.")
            rate.sleep()
        rospy.loginfo("Block Selection - Got block data,")

        # Iterate through blockData and retrieve list of block names
        blockNames = []
        for block_num in range(len(blockData.block_data)):
            block_name = "block" + str(blockData.block_data[block_num].block_number)

            if is_block_reachable(block_name, robot_namespaces):
                blockNames.append(block_name)
                print("Block Selection - Adding", block_name, "as it is reachable.")
            else:
                print("Block Selection - Ignoring", block_name, "as it is unreachable.")

        rospy.loginfo("Block Selection - Block list built.")

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
        rospy.loginfo("Block Selection - Block selection complete. Beginnning publishing.")

        # Publish assignments
        for i in range(max(len(x) for x in goCollect)):
            for j in range(len(robot_namespaces)):
                if i < len(goCollect[j]):
                    # Set End Position
                    block_name = str(goCollect[j][i])

                    end_pos = Pose()

                    orientation_in_euler = [0,90*math.pi/180,0]
                    orientation = tf_conversions.transformations.quaternion_from_euler(orientation_in_euler[0], orientation_in_euler[1], orientation_in_euler[2])
                    
                    end_pos.orientation.x = orientation[0]
                    end_pos.orientation.y = orientation[1]
                    end_pos.orientation.z = orientation[2]
                    end_pos.orientation.w = orientation[3]
                    
                    end_pos.position.z = 0.2

                    if j == 0:
                        end_pos.position.x = 0.25
                        end_pos.position.y = 0
                    else:
                        end_pos.position.x = 0.25
                        end_pos.position.y = 0.5

                    robot_name = str(robot_namespaces[j])
                    
                    try:
                        success = path_service(block_name, end_pos, robot_name)

                        if not(success):
                            rospy.loginfo("Block Selection - Service call returned False.")
                            
                    except rospy.ServiceException as e:
                        rospy.loginfo("Block Selection - Service call failed: %s"%e)

                    

                rate.sleep()

def specific_block_pose(specific_model_name, reference_model_name) -> Pose:
    # Use service to get position of specific block named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    data = model_state_service(specific_model_name, reference_model_name).pose

    # Return ModelState object with position relative to world 
    return data

def is_block_reachable(block_name, robot_namespaces):
    rospy.wait_for_service('inverse_kinematics_reachability')
    inv_kin_is_reachable = rospy.ServiceProxy('inverse_kinematics_reachability', InvKin)

    model_state = ModelState()

    for robot_name in robot_namespaces:
        model_state.pose = specific_block_pose(block_name, robot_name)

        orientation_in_euler = [0,90*math.pi/180,0]
        orientation = tf_conversions.transformations.quaternion_from_euler(orientation_in_euler[0], orientation_in_euler[1], orientation_in_euler[2])
        
        model_state.pose.orientation.x = orientation[0]
        model_state.pose.orientation.y = orientation[1]
        model_state.pose.orientation.z = orientation[2]
        model_state.pose.orientation.w = orientation[3]

        model_state.pose.position.z += 0.15
        
        if inv_kin_is_reachable(model_state):
            return True
        
    return False

if __name__ == '__main__':
    try:
        choose_block()
    except rospy.ROSInterruptException:
        pass
