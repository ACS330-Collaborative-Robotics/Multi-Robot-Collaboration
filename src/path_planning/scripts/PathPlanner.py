# Name: Path Planner Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import InvKin
import PickUp
import Movement

from gazebo_msgs.msg import ModelState
import rospy

class PathPlanner:
    def __init__(self, robot_ns, block_name, end_pos):
        # Setup Objects
        self.invKin = InvKin()
        self.pickUp = PickUp()
        self.pickDown = PickDown()
        self.movement = Movement()