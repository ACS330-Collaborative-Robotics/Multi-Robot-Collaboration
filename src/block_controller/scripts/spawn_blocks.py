#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

import random

def spawner():
    rospy.init_node('block_spawner')

    rospy.wait_for_service('gazebo/spawn_urdf_model')
    urdf_spawner = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)

    f = open('/home/conor/catkin_ws/src/block_controller/urdf/block.urdf')
    urdf = f.read()

    pos = Pose()
    pos.position.x = 1;
    pos.position.y = 1;
    pos.position.z = 1;
    
    urdf_spawner("block", urdf, "blocks", pos, "world")
        

if __name__ == '__main__':
    try:
        spawner()
    except rospy.ROSInterruptException:
        pass
