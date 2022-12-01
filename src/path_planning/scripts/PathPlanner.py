# Name: Path Planner Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

from InvKin import InvKin
from PickUp import PickUp
from PickDown import PickDown
from Movement import Movement

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState

class PathPlanner:
    def __init__(self, robot_ns, block_name, end_pos):
        # Store init data
        self.robot_ns = robot_ns
        self.target_block = block_name
        self.end_pos = end_pos

        # Setup Inverse Kinematics Object
        self.invKin = InvKin(self.robot_ns)

        # Setup Planning Objects with IK object
        self.pickUp = PickUp(self.invKin)
        self.pickDown = PickDown(self.invKin)
        self.movement = Movement(self.invKin)

    def pathPlan(self):
        # Pick up block
        self.pickUp.pick()

        # Move arm
        self.movement.move(self.end_pos)

        # Put down block
        self.pickDown.pick(self.target_block, self.end_pos)

    def specific_block_pos(self, specific_model_name, reference_model_name):
        # TODO: Modify to return Pose object, including rotation
        # Use service to get position of specific block named
        rospy.wait_for_service('gazebo/get_model_state')
        model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        data = model_state_service(specific_model_name, reference_model_name).pose.position

        temp = Pose()

        return temp