#!/usr/bin/env python

# Name: Random block spawner from URDF
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
import numpy as np
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from pathlib import Path

import tf2_ros
import tf

from random import random, randint
from math import pi, cos, sin, floor

def spawner():
    # Setup Node
    rospy.init_node('block_spawner') 

    # Setup URDF spawner service
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    urdf_spawner = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)

    # Open URDF
    f = open(str(Path.home()) + '/catkin_ws/src/block_controller/urdf/block.urdf')
    urdf = f.read()

    # Spawn blocks in radius around each robot base with a minimum and maximum distance

    robot_namespaces = ["mover6_a", "mover6_b"]
    robot_base_coords = getRobotBaseCoordinates(robot_namespaces)

    x_lower_limit = -0.3
    x_upper_limit = 0.3

    robot_a_block_coordinates = [[x_lower_limit, -0.1], [x_lower_limit, 0], [x_lower_limit, 0.05], [x_upper_limit, -0.1], [x_upper_limit, 0], [x_upper_limit, 0.1]]

    y_offset = 0.72

    robot_b_block_coordinates = []
    for block_coordinates in robot_a_block_coordinates:
        robot_b_block_coordinates.append([block_coordinates[0], block_coordinates[1] + y_offset])

    block_coordinates = robot_a_block_coordinates + robot_b_block_coordinates
    
    pos = Pose() # Pose object to be filled randomly
    for block_num in range(len(block_coordinates)):
        # Assign position, offset by robot base coordinates
        pos.position.x = block_coordinates[block_num][0]
        pos.position.y = block_coordinates[block_num][1]
        pos.position.z = 0.01

        quat = tf.transformations.quaternion_from_euler(0, 0, -0.5)
        pos.orientation.x = quat[0]
        pos.orientation.y = quat[1]
        pos.orientation.z = quat[2]
        pos.orientation.w = quat[3]
        
        #urdf_spawner(model_name, model_xml, model_namespace, Pose initial_pose, reference_frame)
        rospy.loginfo(urdf_spawner("block"  + str(block_num), urdf, "blocks", pos, "world").status_message)
        
def getRobotBaseCoordinates(robot_namespaces):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    base_coordinates = []
    for robot_name in robot_namespaces:
        robot_base_coordinates = []
        while not tfBuffer.can_transform("world", robot_name+"/base_link", rospy.Time(0)):
            rospy.logwarn_once("Cannot find robot base transform - spawn_blocks.py. Retrying now.")
            rospy.sleep(0.1)
        
        transform_response = tfBuffer.lookup_transform("world", robot_name+"/base_link", rospy.Time(0))

        robot_base_coordinates.append(transform_response.transform.translation.x)
        robot_base_coordinates.append(transform_response.transform.translation.y)

        base_coordinates.append(robot_base_coordinates)
    
    return base_coordinates

if __name__ == '__main__':
    try:
        spawner()
    except rospy.ROSInterruptException:
        pass