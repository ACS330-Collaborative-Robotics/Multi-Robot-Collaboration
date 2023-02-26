# Name: Pick Up Class Definition
# Authors: Conor Nichols (cjnichols1@sheffield.ac.uk) Joseph Fields (jfields1@sheffield.ac.uk)

import rospy
from path_planner import Movement

class PickUp(Movement.Movement):
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper
    
    def pick(self, block_name):
        """ Pick up specified block

        INPUT: block_name
        OUTPUT: bool Success
        """
        pose = self.serv_helper.getBlockPos(block_name)
        
        # Move 5cm above block
        pose.position.z += 0.05

        # Set End Effector orientation to point downwards using quaternions
        pose.orientation.x = 0
        pose.orientation.y = 1
        pose.orientation.z = 0
        pose.orientation.w = 0
        
        if self.move(pose):
            rospy.loginfo("Path Planner - Succesfully positioned above block.")

    def moveGripper(self, state):
        pass

    def APFpick(self, block_name): #same as pick but self.move is self.APFmove
        """ Pick up specified block

        INPUT: block_name
        OUTPUT: bool Success
        """
        pose = self.serv_helper.getBlockPos(block_name)
        
        # Move 5cm above block
        pose.position.z += 0.05

        # Set End Effector orientation to point downwards using quaternions
        pose.orientation.x = 0
        pose.orientation.y = 1
        pose.orientation.z = 0
        pose.orientation.w = 0
        
        if self.APFmove(pose):
            rospy.loginfo("Path Planner - Succesfully positioned above block.")

