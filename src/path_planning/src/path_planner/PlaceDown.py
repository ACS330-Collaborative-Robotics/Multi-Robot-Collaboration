# Name: Pick Down Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

from path_planner import PickUp
import rospy
import tf_conversions
from math import pi
from path_planner import Movement
class PlaceDown(PickUp.PickUp):
    def __init__(self, serv_helper):
        super().__init__(serv_helper)

    def place(self,target_block, end_pos):
        rospy.loginfo("Place block: %s",target_block)
        pass