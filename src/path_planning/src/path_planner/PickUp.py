# Name: Pick Up Class Definition
# Authors: Conor Nichols (cjnichols1@sheffield.ac.uk) Joseph Fields (jfields1@sheffield.ac.uk)

from path_planner import Movement

import rospy
import tf_conversions
from math import pi

class PickUp(Movement.Movement):
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper
    

    def moveGripper(self, state):
        pass

    def pick(self, block_name): 
        """ Pick up specified block

        INPUT: block_name
        OUTPUT: bool Success
        """
        pose = self.serv_helper.getBlockPos(block_name)
        
        # Move 5cm above block
        pose.position.z += 0.15

        # Set End Effector orientation to point downwards using quaternions
        orientation_in_euler = [0,180*pi/180,0]
        orientation = tf_conversions.transformations.quaternion_from_euler(orientation_in_euler[0], orientation_in_euler[1], orientation_in_euler[2])
        
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        rospy.loginfo("Path Planner - Pick Up - Moving to %s", block_name)
        
        return self.move(pose)

