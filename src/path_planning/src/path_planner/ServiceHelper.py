# Name: ServiceHelper Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
from inv_kinematics.srv import InvKin

from math import atan2, asin

class ServiceHelper:
    def __init__(self, robot_ns):
        self.robot_ns = robot_ns

        # Setup inverse_kinematics service
        rospy.wait_for_service('inverse_kinematics')
        self.inv_kin = rospy.ServiceProxy('inverse_kinematics', InvKin)

        # Setup get_model_state service
        rospy.wait_for_service('gazebo/get_model_state')
        self.model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

    def move(self, pos:Pose):
        """ Move arm to specified position.

        INPUT: geometry_msgs Pose() - Orientation in Euler angles not quaternions

        Uses inverse_kinematics service.
        """

        rospy.wait_for_service('inverse_kinematics')

        # Initialise and fill ArmPos object
        arm_pos = ModelState()
        arm_pos.model_name = self.robot_ns

        arm_pos.pose = pos

        # Call inverse_kinematics service and log ArmPos
        self.inv_kin(arm_pos)

    def getBlockPos(self, specific_model_name:str) -> Pose:
        """ Get block position relative to current robot arm

        INPUT: string specific_model_name
        OUTPUT: gazebo_msgs Pose() - Orientation in Euler angles not quaternions

        Uses gazebo/get_model_state service.
        """

        rospy.wait_for_service('gazebo/get_model_state')

        # Extract Pose() object
        data = self.model_state_service(specific_model_name, self.robot_ns).pose
        
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