#!/usr/bin/env python

# Name: Random block spawner from URDF
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
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

    # Spawn blocks in radius around each robot base with a minimum and maximum distance

    robot_namespaces = ["mover6_a", "mover6_b"]
    robot_base_coords = getRobotBaseCoordinates(robot_namespaces)
    
    min_range = 0.15
    max_range = 0.35
    
    back_reach_maximum = 0.2

    pos = Pose() # Pose object to be filled randomly
    for block_num in range(20):
        # Using angle + distance to select random location within range
        angle = 2*pi*random()
        distance = (max_range-min_range)*random() + min_range

        # Standard 2x2 rotation matrix transformation
        x = cos(angle) * distance + sin(angle) * 0
        y = -sin(angle) * distance + cos(angle) * 0

        # Select a robot base randomly
        robot_num = randint(0, len(robot_base_coords)-1)

        # Assign position, offset by robot base coordinates
        pos.position.x = x + robot_base_coords[robot_num][0]
        pos.position.y = y + robot_base_coords[robot_num][1]
        pos.position.z = 0.01

        # Ensure blocks do not spawn "behind" robot becoming unreachable
        if robot_num == 0 and pos.position.y < robot_base_coords[robot_num][1] - back_reach_maximum:
            pos.position.y = robot_base_coords[robot_num][1] - back_reach_maximum
        
        if robot_num == 1 and pos.position.y > robot_base_coords[robot_num][1] + back_reach_maximum:
            pos.position.y = robot_base_coords[robot_num][1] + back_reach_maximum

        # quaternion roation w x y z
        pos.orientation.w = 2*random() - 1
        pos.orientation.x = 0 # a - Roll
        pos.orientation.y = 0 # Lengthway vertically
        pos.orientation.z = 2*random() - 1 # Flat rotation
        
        #urdf_spawner(model_name, model_xml, model_namespace, Pose initial_pose, reference_frame)
        print(urdf_spawner("block"  + str(block_num), urdf, "blocks", pos, "world"))
        
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
