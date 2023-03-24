#!/usr/bin/python3

# Name: Block Position Updater
# Author: Joseph Fields (jfields1@sheffield.ac.uk)

import rospy
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from pathlib import Path
from block_controller.msg import Blocks,Block
import tf2_ros

from random import random, randint
from math import pi, cos, sin
import tf_conversions

blockData_cam = None
blockData = None

def update():
    # Setup Node
    rospy.init_node('block_updater') 

    #subscribe to camera and simulation positions
    rospy.Subscriber('/blocks_pos_cam', Blocks, callback_cam)
    rospy.Subscriber('/blocks_pos', Blocks, callback)
    # Setup set model state service
    rospy.wait_for_service('gazebo/set_model_state')
    block_updater = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

    #wait for data from camera
    while ((blockData is None) or (blockData_cam is None)) and not(rospy.is_shutdown()):
        rospy.loginfo("Block Updater - Waiting for block and camera data.")
        rospy.sleep(0.1)
    rospy.loginfo("Block Updater - Got block and camera data.")

    # Spawn blocks in radius around each robot base with a minimum and maximum distance
    while not rospy.is_shutdown():

        Tol=0.15
        TolAng=1
        for i in range(len(blockData.block_data)):
            fail=0

            if blockData_cam.block_data[i].x >= blockData.block_data[i].x+Tol or blockData_cam.block_data[i].x <= blockData.block_data[i].x-Tol:
                fail=1
                print("x fail")
            if blockData_cam.block_data[i].y >= blockData.block_data[i].y+Tol or blockData_cam.block_data[i].y <= blockData.block_data[i].y-Tol:
                fail=1
                print("y fail")
            if blockData_cam.block_data[i].z >= blockData.block_data[i].z+Tol or blockData_cam.block_data[i].z <= blockData.block_data[i].z-Tol:
                fail=1
                print("z fail")
            
            if blockData_cam.block_data[i].a >= blockData.block_data[i].a+TolAng or blockData_cam.block_data[i].a <= blockData.block_data[i].a-TolAng:
                fail=1
                print("a fail")
            if blockData_cam.block_data[i].b >= blockData.block_data[i].b+TolAng or blockData_cam.block_data[i].b <= blockData.block_data[i].b-TolAng:
                fail=1
                print("b fail")
            if blockData_cam.block_data[i].c >= blockData.block_data[i].c+TolAng or blockData_cam.block_data[i].c <= blockData.block_data[i].c-TolAng:
                fail=1
                print("c fail")

            if fail==1:
                pos=Pose()
                model=ModelState()
                model.model_name="block"+ str(blockData.block_data[i].block_number)

                pos.position.x=blockData_cam.block_data[i].x
                pos.position.y=blockData_cam.block_data[i].y
                pos.position.z=blockData_cam.block_data[i].z
                a=blockData_cam.block_data[i].a
                b=blockData_cam.block_data[i].b
                c=blockData_cam.block_data[i].c

                orientation = tf_conversions.transformations.quaternion_from_euler(a,b,c)
                pos.orientation.x = orientation[0]
                pos.orientation.y = orientation[1]
                pos.orientation.z = orientation[2]
                pos.orientation.w = orientation[3]

                model.pose=pos
                model.twist.linear.x=0
                model.twist.linear.y=0
                model.twist.linear.z=0

                model.twist.angular.x=0
                model.twist.angular.y=0
                model.twist.angular.z=0

                model.reference_frame="world"

                block_updater(model)
    rospy.sleep(0.5)
            

def callback_cam(data):
    global blockData_cam
    blockData_cam = data       
def callback(data):
    global blockData
    blockData = data

if __name__ == '__main__':
    try:
        update()
    except rospy.ROSInterruptException:
        pass
