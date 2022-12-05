# Name: Pick Up Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy

class PickUp:
    def __init__(self, serv_helper, movement):
        self.serv_helper = serv_helper
        self.movement = movement
    
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
        
        if self.movement.move(pose):
            rospy.loginfo("Path Planner - Succesfully positioned above block.")

    def moveGripper(self, state):
        pass