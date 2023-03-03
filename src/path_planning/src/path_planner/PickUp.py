# Name: Pick Up Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from path_planner import Movement
from block_controller.msg import Blocks

class PickUp(Movement.Movement):
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper
    
    def pick(self, block_name, data):
        rospy.loginfo(rospy.get_caller_id() + "%s", data.block_data)

        """ Pick up specified block

        INPUT: block_name
        OUTPUT: bool Success
        """
        pose = self.serv_helper.getBlockPos(block_name)
        
        # Move 13cm above block
        pose.position.z += 0.13

        # Set End Effector orientation to point downwards using quaternions
        pose.orientation.x = data.block_data.z
        pose.orientation.y = 1
        pose.orientation.z = 0
        pose.orientation.w = 0
        print("Path Planner - Pick Up - Moving to ", block_name)
        
        self.move(pose)

    def moveGripper(self, state):
        pass