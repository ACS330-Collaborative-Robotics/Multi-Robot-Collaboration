#!/usr/bin/env python

# Name: Random block spawner from URDF
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from pathlib import Path
from block_controller.msg import Blocks,Block
import tf2_ros

from random import random, randint
from math import pi, cos, sin
import tf_conversions

def spawner():
    # Setup Node
    rospy.init_node('block_spawner') 

    rospy.Subscriber('/blocks_pos', Blocks, callback)
    # Setup URDF spawner service
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    urdf_spawner = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)

    # Open URDF
    f = open(str(Path.home()) + '/catkin_ws/src/block_controller/urdf/block.urdf')
    urdf = f.read()

    #wait for data from camera
    while (blockData is None) and not(rospy.is_shutdown()):
        rospy.loginfo("Block Spawner - Waiting for camera data.")
        rospy.sleep(0.1)
    rospy.loginfo("Block Spawner - Got camera data,")

    # Spawn blocks in radius around each robot base with a minimum and maximum distance
    
    pos = Pose() # Pose object to be filled randomly
    for i in range(length(BlockData)):
        pos.position.x=BlockData.block_data[i].x
        pos.position.y=BlockData.block_data[i].y
        pos.position.z=BlockData.block_data[i].z
        a=BlockData.block_data[i].a
        b=BlockData.block_data[i].b
        c=BlockData.block_data[i].c

        orientation = tf_conversions.transformations.quaternion_from_euler(a,b,c)
        pos.orientation.x = orientation[0]
        pos.orientation.y = orientation[1]
        pos.orientation.z = orientation[2]
        pos.orientation.w = orientation[3]

        print(urdf_spawner("block"  + str(BlockData.block_data.block_number), urdf, "blocks", pos, "world"))
        
def callback(data):
    global blockData
    blockData = data

def getRobotBaseCoordinates(robot_namespaces):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    base_coordinates = []
    for robot_name in robot_namespaces:
        robot_base_coordinates = []
        while not tfBuffer.can_transform("world", robot_name+"_base", rospy.Time(0)):
            print("Cannot find robot base transform - spawn_blocks.py. Retrying now.")
            rospy.sleep(0.1)
        
        transform_response = tfBuffer.lookup_transform("world", robot_name+"_base", rospy.Time(0))

        robot_base_coordinates.append(transform_response.transform.translation.x)
        robot_base_coordinates.append(transform_response.transform.translation.y)

        base_coordinates.append(robot_base_coordinates)
    
    return base_coordinates

if __name__ == '__main__':
    try:
        spawner()
    except rospy.ROSInterruptException:
        pass
