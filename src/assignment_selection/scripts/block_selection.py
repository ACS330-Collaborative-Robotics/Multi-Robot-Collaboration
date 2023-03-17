#!/usr/bin/env python

# Name: block_slection
# Author: Tom Richards (tomtommrichards@gmail.com), Conor Nichols (cjnichols1@sheffield.ac.uk), Annanthavel Santhanavel Ramesh(asanthanavelramesh1@sheffield.ac.uk)

import rospy

from gazebo_msgs.srv import GetModelState
from block_controller.msg import Blocks
from path_planning.srv import PathPlan
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import tf
import tf2_ros
import math
from operator import itemgetter
import tf2_ros
import tf_conversions

# Global variable to store blockData as it appears from subscriber
blockData = None

def callback(data):
    global blockData
    blockData = data

def choose_block():
    # Setup block_pos listener
    rospy.Subscriber('/blocks_pos', Blocks, callback)

    # Setup path_planner service
    rospy.wait_for_service('path_planner')
    path_service = rospy.ServiceProxy('path_planner', PathPlan)

    # Declare ROS Node name
    rospy.init_node('block_selector')

    # Define robot namespaces being used - also defines number of robots
    robot_namespaces = ["mover6_a", "mover6_b"]
    robot_base_coords = getRobotBaseCoordinates(robot_namespaces)

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
            blockNames.append("block" + str(blockData.block_data[block_num].block_number))
        rospy.loginfo("Block Selection - Block list built.")

        print(blockNames)

        ## ////////////////////////////////////////////
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
        rospy.loginfo("Block Selection - Block selection complete. Beginnning publishing.")
        
        ## ////////////////////////////////////////////
        
        n = len(blockNames) #num of blocks
        layers = math.ceil(n/3) #num of layers
        tower_pos = [] #this has to be a 3 column * layers(value) matrix
        h=0.1 #height of blocks
        a=0
        b=0
        c=0

        #generate coordinates
        for i in range(layers):
            w=0.25 #width of blocks
            home_pos = [w,0.25,h,a,b,c]
            for j in range(3):
                home_pos = [w,0.25,h,a,b,c]
                tower_pos.append(home_pos)
                w=w+0.2
            h=h+0.1

            if c==0:
                c=-90*(math.pi/180)
            elif c==-90*(math.pi/180):
                c=0

        # Publish assignments
        for i in range(len(tower_pos)):
            block_name = str(blockNames[i])

            end_pos = Pose()
            end_pos.position.x = tower_pos[i][0]
            end_pos.position.y = tower_pos[i][1]
            end_pos.position.z = tower_pos[i][2]

            quat = tf.transformations.quaternion_from_euler(tower_pos[i][3],tower_pos[i][4],tower_pos[i][5])
            end_pos.orientation.x = quat[0]
            end_pos.orientation.y = quat[1]
            end_pos.orientation.z = quat[2]
            end_pos.orientation.w = quat[3]

            robot_name = str(robot_namespaces[0])
            ## ////////////////////////////////////////////
                    
            try:
                success = path_service(block_name, end_pos, robot_name)

                if not(success):
                    rospy.loginfo("Block Selection - Service call returned False.")
                            
            except rospy.ServiceException as e:
                rospy.loginfo("Block Selection - Service call failed: %s"%e)

                    

                rate.sleep()

def specific_block_pos(specific_model_name, reference_model_name):
    # Use service to get position of specific block named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    data = model_state_service(specific_model_name, reference_model_name).pose.position

    # Return ModelState object with position relative to world 
    return [data.x, data.y, data.z]

def getRobotBaseCoordinates(robot_namespaces):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    base_coordinates = []
    for robot_name in robot_namespaces:
        robot_base_coordinates = []
        while not tfBuffer.can_transform("world", robot_name+"_base", rospy.Time(0)) and not rospy.is_shutdown():
            print("Cannot find robot base transform - spawn_blocks.py. Retrying now.")
            rospy.sleep(0.1)
        
        transform_response = tfBuffer.lookup_transform("world", robot_name+"_base", rospy.Time(0))

        robot_base_coordinates.append(transform_response.transform.translation.x)
        robot_base_coordinates.append(transform_response.transform.translation.y)

        base_coordinates.append(robot_base_coordinates)

if __name__ == '__main__':
    try:
        choose_block()
    except rospy.ROSInterruptException:
        pass
