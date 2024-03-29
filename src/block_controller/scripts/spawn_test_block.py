#!/usr/bin/env python

# Name: Spawn single block at specified coordinates
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from pathlib import Path

from random import random

def spawner():
    # Setup Node
    rospy.init_node('block_spawner') 

    # Setup model removal service
    rospy.wait_for_service('gazebo/delete_model')
    model_remover = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

    model_remover("block0")

    # Setup URDF spawner service
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    urdf_spawner = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)

    # Open URDF
    f = open(str(Path.home()) + '/catkin_ws/src/block_controller/urdf/block.urdf')
    urdf = f.read()
    
    pos = Pose() # Pose object to be filled randomly
    block_num = 0

    # Assign position, offset by robot base coordinates
    pos.position.x = 0.2
    pos.position.y = 0.1
    pos.position.z = 0.01

    # quaternion roation w x y z
    pos.orientation.w = 2*random() - 1
    pos.orientation.x = 0 # a - Roll
    pos.orientation.y = 0 # Lengthway vertically
    pos.orientation.z = 2*random() - 1 # Flat rotation
    
    #urdf_spawner(model_name, model_xml, model_namespace, Pose initial_pose, reference_frame)
    rospy.loginfo(urdf_spawner("block"  + str(block_num), urdf, "blocks", pos, "world").status_message)

if __name__ == '__main__':
    try:
        spawner()
    except rospy.ROSInterruptException:
        pass
