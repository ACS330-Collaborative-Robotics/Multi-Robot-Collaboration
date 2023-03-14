#!/usr/bin/env python

# Name: Random block spawner from URDF
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from pathlib import Path

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
    robot_base_coords = [[0,0], [0, 0.5]]

    min_range = 0.15
    max_range = 0.35

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
        if robot_num == 0 and pos.position.y < -0.2:
            pos.position.y = -0.2
        
        if robot_num == 1 and pos.position.y > 0.7:
            pos.position.y = 0.7

        # quaternion roation w x y z
        pos.orientation.w = 2*random() - 1
        pos.orientation.x = 0 # a - Roll
        pos.orientation.y = 0 # Lengthway vertically
        pos.orientation.z = 2*random() - 1 # Flat rotation
        
        #urdf_spawner(model_name, model_xml, model_namespace, Pose initial_pose, reference_frame)
        print(urdf_spawner("block"  + str(block_num), urdf, "blocks", pos, "world"))
        
if __name__ == '__main__':
    try:
        spawner()
    except rospy.ROSInterruptException:
        pass
