#!/usr/bin/env python

# http://docs.ros.org/en/noetic/api/gazebo_msgs/html/index-msg.html

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from pathlib import Path

from random import random

def spawner():
    rospy.init_node('block_spawner')

    rospy.wait_for_service('gazebo/spawn_urdf_model')
    urdf_spawner = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)

    f = open(str(Path.cwd()) + '/src/block_controller/urdf/block.urdf')
    urdf = f.read()

    pos = Pose()
    for block_num in range(5):
        # position x y z
        pos.position.x = 2*random() - 1
        pos.position.y = 2*random() - 1
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
