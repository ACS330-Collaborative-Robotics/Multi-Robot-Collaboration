#!/usr/bin/python3

# Name: Block Position Updater
# Author: Joseph Fields (jfields1@sheffield.ac.uk)

import rospy
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from block_controller.msg import Blocks

from block_controller.srv import UpdateBlocks
import tf_conversions

blockData_cam = None
#to call, send True or False 
def update(data):
    remainingTags=[0,1,10,11,12,13,14,15,16,17,18,19]
    count=0

    # Setup set model state service
    rospy.wait_for_service('gazebo/set_model_state')
    block_updater = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

    #wait for data from camera (MIGHT NOT BE NEEDED)
    while (blockData_cam is None) and not rospy.is_shutdown():
        rospy.loginfo("Block Updater - Waiting for block and camera data.")
        rospy.sleep(0.1)

    rospy.loginfo("Block Updater - Got block and camera data.")
    
    while len(remainingTags) > 0 and not rospy.is_shutdown():
        for i in range(len(blockData_cam.block_data)):
            pos = Pose()
            model = ModelState()
            blocknumber = blockData_cam.block_data[i].block_number
            model.model_name = "block" + str(blocknumber)

            pos.position.x = blockData_cam.block_data[i].x
            pos.position.y = blockData_cam.block_data[i].y
            pos.position.z = blockData_cam.block_data[i].z
            a = blockData_cam.block_data[i].a
            b = blockData_cam.block_data[i].b
            c = blockData_cam.block_data[i].c

            orientation = tf_conversions.transformations.quaternion_from_euler(a,b,c)
            pos.orientation.x = orientation[0]
            pos.orientation.y = orientation[1]
            pos.orientation.z = orientation[2]
            pos.orientation.w = orientation[3]

            model.pose = pos
            #model.reference_frame = "world" #might cause issues
            block_updater(model)

            if remainingTags.count(blocknumber) == 1:
                remainingTags.remove(blocknumber)

            count = count+1

        if(count < 25):
            rospy.loginfo("Block Updater - Tags not found: %s",remainingTags)
        else:
            rospy.logwarn("Block Updater - Tags STILL not found: %s. Are tags in frame?",remainingTags)

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
