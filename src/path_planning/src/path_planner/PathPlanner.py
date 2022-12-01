# Name: Path Planner Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

from path_planner import PickUp, PickDown, InvKin, Movement

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState

from math import atan2, asin

class PathPlanner:
    def __init__(self, robot_ns, block_name, end_pos):
        # Store init data
        self.robot_ns = robot_ns
        self.target_block = block_name
        self.end_pos = end_pos

        # Setup Inverse Kinematics Object
        self.invKin = InvKin.InvKin(self.robot_ns)

        # Setup Planning Objects with IK object
        self.pickUp = PickUp.PickUp(self.invKin)
        self.pickDown = PickDown.PickDown(self.invKin)
        self.movement = Movement.Movement(self.invKin)

    def pathPlan(self):
        # Pick up block
        self.pickUp.pick()

        # Move arm
        self.movement.move(self.end_pos)

        # Put down block
        self.pickDown.pick(self.target_block, self.end_pos)
    
    def getBlockPos(self, specific_model_name: str, reference_model_name:str) -> Pose:
        """ Get block position relative to specified model.

        INPUT: string specific_model_name, string reference model_name
        OUTPUT: gazebo_msgs Pose() - Orientation in Euler angles not quaternions

        Uses gazebo/get_model_state service.
        """
        # Use service to get position of specific block named
        rospy.wait_for_service('gazebo/get_model_state')
        model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

        # Extract Pose() object
        data = model_state_service(specific_model_name, reference_model_name).pose
        
        # Get quaternion rotation data
        w = data.orientation.w
        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z

        # Convert quaternion to pitch, roll, yaw
        data.orientation.x = atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z) # Pitch - a
        data.orientation.y = atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z) # Roll - b
        data.orientation.z = asin(2*x*y + 2*z*w) # Yaw - c
        data.orientation.w = 0

        return data