# Name: Pick Up Class Definition
# Authors: Conor Nichols (cjnichols1@sheffield.ac.uk) Joseph Fields (jfields1@sheffield.ac.uk)

import rospy
import tf_conversions
from math import pi
from path_planner import Movement

class PickUp(Movement.Movement):
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper
    

    def moveGripper(self, state):
        pass

    def pick(self, block_name): #same as pick but self.move is self.APFmove
        """ Pick up specified block

        INPUT: block_name
        OUTPUT: bool Success
        """
        pose = self.serv_helper.getBlockPos(block_name)
        
        # Move 5cm above block
        pose.position.z += 0.1

        # Set End Effector orientation to point downwards using quaternions
        orientation_in_euler = [0,180*pi/180,0]
        orientation = tf_conversions.transformations.quaternion_from_euler(orientation_in_euler[0], orientation_in_euler[1], orientation_in_euler[2])
        
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        print("Path Planner - Pick Up - Moving to ", block_name)
        
        self.move(pose)

