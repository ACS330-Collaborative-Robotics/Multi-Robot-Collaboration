#!/usr/bin/env python

# Name: block_slection
# Author: Tom Richards (tomtommrichards@gmail.com), Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
import math
from gazebo_msgs.srv import GetModelState
from operator import itemgetter
from std_msgs.msg import String
from block_controller.msg import Blocks

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
    
    # Setup publishers for each robot
    pub = []
    for robot in robot_namespaces:
        pub.append(rospy.Publisher(robot + "/next_block", String , queue_size=2))

    # Declare ROS Node name
    rospy.init_node('block_selector')

    # Set Loop rate
    rate = rospy.Rate(0.5)

    ## Making array of block names ##

    # Wait for blockData to read in by subscriber
    while blockData is None:
        rospy.loginfo("Waiting for data.")
        rate.sleep()
    rospy.loginfo("Got block data,")

    # Iterate through blockData and retrieve list of block names
    blockNames = []
    for block_num in range(len(blockData.block_data)):
        blockNames.append("block" + str(blockData.block_data[block_num].block_number))
    rospy.loginfo("Block list built.")

    # Getting distance from each robot to blocks and sellecting the smallest
    roboColect = []
    for blockName in blockNames:     
        reldist = []
        for robot in robot_namespaces:
            temp =  specific_block_pos(blockName, robot)
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
    rospy.loginfo("Block selection complete. Beginnning publishing.")

    # Publish assignments
    for i in range(max(len(x) for x in goCollect)):
        for j in range(len(pub)):
            try:
                pub[j].publish(goCollect[j][i])
                rospy.loginfo(robot_namespaces[j] + " " + goCollect[j][i])
            except:
                pass

            rate.sleep()

def specific_block_pos(specific_model_name, reference_model_name):
    # Use service to get position of specific block named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    data = model_state_service(specific_model_name, reference_model_name).pose.position

    # Return ModelState object with position relative to world 
    return [data.x, data.y, data.z]

if __name__ == '__main__':
    try:
        choose_block()
    except rospy.ROSInterruptException:
        pass
