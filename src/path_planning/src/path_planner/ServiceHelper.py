# Name: ServiceHelper Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
import tf2_ros
import tf_conversions

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
from tf2_geometry_msgs import PoseStamped
from inv_kinematics.srv import InvKin
from std_msgs.msg import Bool

from math import pi

class ServiceHelper:
    def __init__(self, robot_ns):
        self.robot_ns = robot_ns

        # Setup inverse_kinematics service
        rospy.wait_for_service('/inverse_kinematics')
        self.inv_kin = rospy.ServiceProxy('/inverse_kinematics', InvKin)

        rospy.wait_for_service('/inverse_kinematics_reachability')
        self.inv_kin_reachable = rospy.ServiceProxy('/inverse_kinematics_reachability', InvKin)

        # Setup get_model_state service
        rospy.wait_for_service('/gazebo/get_model_state')
        self.model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Setup tf2
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Setup gripper publisher
        self.gripper_publisher = rospy.Publisher("gripper_state", Bool, queue_size=10)

    def move(self, pos:Pose, final_link_name):
        """ Move arm to specified position.

        INPUT: geometry_msgs Pose() - Orientation as quaternions

        Uses inverse_kinematics service.
        """

        rospy.wait_for_service('/inverse_kinematics')

        #rospy.loginfo("Path Planner - Service Helper - Calling ik for %s", self.robot_ns)

        # Initialise and fill ArmPos object
        arm_pos = ModelState()
        arm_pos.model_name = self.robot_ns
        arm_pos.reference_frame = final_link_name

        arm_pos.pose = pos

        # Call inverse_kinematics service and log ArmPos
        return self.inv_kin(arm_pos).success
    
    def moveGripper(self, state:bool):
        self.gripper_publisher.publish(state)
        
        #rospy.loginfo("Path Planner - Service Helper - Gripper set to state %i.", state)

    def getBlockPos(self, specific_model_name:str) -> Pose:
        """ Get block position relative to current robot arm

        INPUT: string specific_model_name
        OUTPUT: gazebo_msgs Pose() - Orientation in Euler angles not quaternions

        Uses gazebo/get_model_state service.
        """
        # TODO: Replace with data from /blocks

        rospy.wait_for_service('/gazebo/get_model_state')

        # Extract Pose() object
        data = self.model_state_service(specific_model_name, "world").pose

        return data

    def frameConverter(self, target_frame:str, reference_frame:str, goal_pose:Pose) -> Pose:
        # Setup time stamped pose object
        start_pose = PoseStamped()
        start_pose.pose = goal_pose

        orientation_in_quaternion = [start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w]
        orientation_in_euler = tf_conversions.transformations.euler_from_quaternion(orientation_in_quaternion)

        rospy.loginfo("Frame Converter - Start pose:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f", start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z, orientation_in_euler[0]*180/pi, orientation_in_euler[1]*180/pi, orientation_in_euler[2]*180/pi)

        start_pose.header.frame_id = reference_frame
        start_pose.header.stamp = rospy.get_rostime()

        # Convert from world frame to robot frame using tf2
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                new_pose = self.tfBuffer.transform(start_pose, target_frame+"/base_link")
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("Error - Frame converter in Path Planner ServiceHelper.py failed. Retrying now.")
                rate.sleep()
                continue
        
        orientation_in_quaternion = [new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w]
        orientation_in_euler = tf_conversions.transformations.euler_from_quaternion(orientation_in_quaternion)
        
        rospy.loginfo("Frame Converter - New pose:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f", new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z, orientation_in_euler[0]*180/pi, orientation_in_euler[1]*180/pi, orientation_in_euler[2]*180/pi)

        return new_pose.pose
    

    def fix_block_pose_orientation(self, pose):
        model_state = ModelState()
        model_state.pose = pose

        block_orientation_quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        block_orientation_euler = tf_conversions.transformations.euler_from_quaternion(block_orientation_quaternion)

        for angle_offset in [0, -pi, pi]:
            print(angle_offset)
            orientation_euler = [0, pi, block_orientation_euler[2]+angle_offset]
            orientation_quaternion = tf_conversions.transformations.quaternion_from_euler(orientation_euler[0], orientation_euler[1], orientation_euler[2])
            
            model_state.pose.orientation.x = orientation_quaternion[0]
            model_state.pose.orientation.y = orientation_quaternion[1]
            model_state.pose.orientation.z = orientation_quaternion[2]
            model_state.pose.orientation.w = orientation_quaternion[3]

            model_state.pose = self.frameConverter(self.robot_ns, "world", model_state.pose)

            # Test at two heights above the block
            model_state.pose.position.z += 0.10
            if self.inv_kin_reachable(model_state).success:
                print(angle_offset)

                model_state.pose.position.z += 0.05
                if self.inv_kin_reachable(model_state).success:
                    print(angle_offset)
                    return angle_offset
        
        return False
