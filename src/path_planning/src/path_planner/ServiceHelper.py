# Name: ServiceHelper Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
import tf2_ros

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
from tf2_geometry_msgs import PoseStamped
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

        # Setup tf2
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def move(self, pos:Pose, final_link_name):
        """ Move arm to specified position.

        INPUT: geometry_msgs Pose() - Orientation as quaternions

        Uses inverse_kinematics service.
        """

        rospy.wait_for_service('inverse_kinematics')

        rospy.loginfo("Path Planner - Service Helper - Calling ik for %s", self.robot_ns)

        # Initialise and fill ArmPos object
        arm_pos = ModelState()
        arm_pos.model_name = self.robot_ns
        arm_pos.reference_frame = final_link_name

        arm_pos.pose = pos

        # Call inverse_kinematics service and log ArmPos
        return self.inv_kin(arm_pos).success
    
    def getBlockPos(self, specific_model_name:str) -> Pose:
        """ Get block position relative to current robot arm

        INPUT: string specific_model_name
        OUTPUT: gazebo_msgs Pose() - Orientation in Euler angles not quaternions

        Uses gazebo/get_model_state service.
        """
        # TODO: Replace with data from /blocks

        rospy.wait_for_service('gazebo/get_model_state')

        # Extract Pose() object
        data = self.model_state_service(specific_model_name, "world").pose

        return data

    def frameConverter(self, target_frame:str, reference_frame:str, goal_pose:Pose) -> Pose:
        # Setup time stamped pose object
        start_pose = PoseStamped()
        start_pose.pose = goal_pose

        rospy.loginfo("Frame Converter - Start pose:\t%.2f\t%.2f\t%.2f", start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z)

        start_pose.header.frame_id = reference_frame
        start_pose.header.stamp = rospy.get_rostime()

        # Convert from world frame to robot frame using tf2
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                new_pose = self.tfBuffer.transform(start_pose, target_frame+"_base")
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Error - Frame converter in Path Planner ServiceHelper.py failed. Retrying now.")
                rate.sleep()
                continue
        
        rospy.loginfo("Frame Converter - New pose:\t%.2f\t%.2f\t%.2f", new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z)

        return new_pose.pose
