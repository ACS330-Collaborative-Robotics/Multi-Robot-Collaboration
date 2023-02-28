#!/usr/bin/env python

# Name: Random block spawner from URDF
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from pathlib import Path

from random import random

def spawner():
    # Setup Node
    rospy.init_node('block_spawner') 

    # Setup URDF spawner service
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    urdf_spawner = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)

    # Open URDF
    f = open(str(Path.home()) + '/catkin_ws/src/block_controller/urdf/block.urdf')
    urdf = f.read()

    pos = Pose() # Pose object to be filled randomly
    for block_num in range(20):
        # position x y z
        # square around origin of width and height, x_range and y_range respectively
        x_range = 0.6
        y_range = 1

        pos.position.x = x_range*random() - x_range/2
        pos.position.y = y_range*random() - y_range/4
        pos.position.z = 0.01

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
