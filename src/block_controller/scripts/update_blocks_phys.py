#!/usr/bin/python3

# Name: Block Position Updater
# Author: Joseph Fields (jfields1@sheffield.ac.uk)

import rospy
import numpy
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from block_controller.msg import Blocks

from block_controller.srv import UpdateBlocks
import tf_conversions

blockData_cam = None
#to call, send True or False 
def update(data):

    block_storage_x = numpy.ones(10) #coords to move blocks to if their tags are not found
    block_storage_x = [x * 1 for x in block_storage_x] 
    block_storage_y = range(10,65,5)
    block_storage_y = [y / 100 for y in block_storage_y]

    TagList=[10,11,12,13,14,15,16,17,18,19]
    remainingTags=TagList#list of tags, found tags are removed from list
    count=0

    # Setup set model state service
    rospy.wait_for_service('gazebo/set_model_state')
    block_mover = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

    #wait for data from camera 
    while (blockData_cam is None) and not rospy.is_shutdown():
        rospy.loginfo("Block Updater - Waiting for block and camera data.")
        rospy.sleep(0.05)

    rospy.loginfo("Block Updater - Got block and camera data.")
    
    while len(remainingTags) > 0 and not rospy.is_shutdown():
        pos = Pose()
        model = ModelState()
        for i in range(len(blockData_cam.block_data)):

            blocknumber = blockData_cam.block_data[i].block_number

            if remainingTags.count(blocknumber) == 1: #only move once
                model.model_name = "block" + str(blocknumber)
                pos.position.x = blockData_cam.block_data[i].x
                pos.position.y = blockData_cam.block_data[i].y
                pos.position.z = blockData_cam.block_data[i].z
                a = 0 #blockData_cam.block_data[i].a #blocks should always be flat
                b = 0 #blockData_cam.block_data[i].b
                c = blockData_cam.block_data[i].c + 1.5708 #manual rotation 

                orientation = tf_conversions.transformations.quaternion_from_euler(a,b,c)
                pos.orientation.x = orientation[0]
                pos.orientation.y = orientation[1]
                pos.orientation.z = orientation[2]
                pos.orientation.w = orientation[3]

                model.pose = pos
                block_mover(model) #move block
                remainingTags.remove(blocknumber) #remove from list

        if(count < 5): #debug logging 
            if len(remainingTags) == 0:
                rospy.loginfo("Block Updater - All blocks updated")
            else:
                rospy.loginfo("Block Updater - Tags not found: %s", remainingTags)
                rospy.sleep(0.05)
        else:
            rospy.logerr("Block Updater - Tags STILL not found: %s. Moving lost tags to storage", remainingTags)
            
            for i in range(len(TagList)): #move blocks that aren't found to storage
                if remainingTags.count(TagList[i])==1:
                    model.model_name = "block" + str(TagList[i])
                    pos.position.x = block_storage_x[i]
                    pos.position.y = block_storage_y[i]
                    pos.position.z = 0.01
  
                    pos.orientation.x = 1
                    pos.orientation.y = 0
                    pos.orientation.z = 0
                    pos.orientation.w = 0

                    model.pose = pos
                    block_mover(model)

            return True
        
        count = count+1

    return True
            
def callback(data):
    global blockData_cam
    blockData_cam = data   

def update_server():
    rospy.init_node("block_update_server")
    s = rospy.Service('block_update', UpdateBlocks, update)

    #subscribe to camera positions
    rospy.Subscriber('/blocks_pos_cam', Blocks, callback)

    rospy.spin()
        

if __name__ == '__main__':
    try:
        update_server()
    except rospy.ROSInterruptException:
        pass
