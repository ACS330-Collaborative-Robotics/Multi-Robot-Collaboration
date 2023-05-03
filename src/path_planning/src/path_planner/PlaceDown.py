# Name: Pick Down Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

from path_planner import PickUp

import rospy
import tf_conversions
from math import pi

class PlaceDown(PickUp.PickUp):
    def __init__(self, serv_helper):
        super().__init__(serv_helper)

    def place(self, end_pose):
        """ Place down specified block

        INPUT: end_pose
        OUTPUT: bool Success
        """
        
        block_orientation_quaternion = [end_pose.orientation.x, end_pose.orientation.y, end_pose.orientation.z, end_pose.orientation.w]
        block_orientation_euler = tf_conversions.transformations.euler_from_quaternion(block_orientation_quaternion)

        # Set End Effector orientation to point downwards using quaternions
        orientation_in_euler = [0, pi, block_orientation_euler[2]]
        orientation = tf_conversions.transformations.quaternion_from_euler(orientation_in_euler[0], orientation_in_euler[1], orientation_in_euler[2])
        
        end_pose.orientation.x = orientation[0]
        end_pose.orientation.y = orientation[1]
        end_pose.orientation.z = orientation[2]
        end_pose.orientation.w = orientation[3]

        # Move above drop point
        rospy.loginfo("Path Planner - %s - Place Down - Moving above location.", self.serv_helper.robot_ns)
        end_pose.position.z += 0.20
        if not self.move(end_pose, True):
            return False
        rospy.sleep(2)
        
        # Move down onto drop point
        rospy.loginfo("Path Planner - %s - Place Down - Lowering block.", self.serv_helper.robot_ns)
        end_pose.position.z -= 0.10
        if not self.move(end_pose, False):
            return False
        
        # Open Gripper
        rospy.loginfo("Path Planner - %s - Place Down - Opening Gripper.", self.serv_helper.robot_ns)
        self.serv_helper.moveGripper(1)
        rospy.sleep(3)

        # Move away from block
        rospy.loginfo("Path Planner - %s - Place Down - Lifting up.", self.serv_helper.robot_ns)
        end_pose.position.z += 0.05
        if not self.move(end_pose, False):
            return False
        
        # Close Gripper
        rospy.loginfo("Path Planner - %s - Place Down - Closing Gripper.", self.serv_helper.robot_ns)
        self.serv_helper.moveGripper(0)
        rospy.sleep(2)

        return True
