#!/usr/bin/env python

# Name: Random block spawner from URDF
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
import numpy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from pathlib import Path

import tf2_ros

from random import random, randint
from math import pi, cos, sin

def spawner():
    # Setup Node
    rospy.init_node('block_spawner') 
    # Setup URDF spawner service
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    urdf_spawner = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)

    # Open URDF
    f = open(str(Path.home()) + '/catkin_ws/src/block_controller/urdf/block.urdf')
    urdf = f.read()

    # Spawn blocks in set pattern outside workspace

    block_x = numpy.ones(10)
    block_x = [x * 1 for x in block_x] 

    block_y = range(10,65,5)
    block_y = [y / 100 for y in block_y] 

    block_num = range(10,20)
    
    pos = Pose() 
    for block_count in range(len(block_x)):
        # Assign position
        pos.position.x = block_x[block_count]
        pos.position.y = block_y[block_count]
        pos.position.z = 0.01

        # quaternion roation w x y z
        pos.orientation.w = 1
        pos.orientation.x = 0 # a - Roll
        pos.orientation.y = 0 # Lengthway vertically
        pos.orientation.z = 0# Flat rotation
        
        #urdf_spawner(model_name, model_xml, model_namespace, Pose initial_pose, reference_frame)
        print(urdf_spawner("block"  + str(block_num[block_count]), urdf, "blocks", pos, "world"))
        
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
