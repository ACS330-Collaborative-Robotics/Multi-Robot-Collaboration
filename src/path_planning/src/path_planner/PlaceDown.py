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

        # Move above block
        rospy.loginfo("Path Planner - Place Down - Moving above.")
        end_pose.position.z += 0.15
        if not self.move(end_pose, True):
            return False
        
        # Move down onto block
        rospy.loginfo("Path Planner - Place Down - Lowering onto.")
        end_pose.position.z -= 0.07
        if not self.move(end_pose, False):
            return False
        
        # Open Gripper
        rospy.logwarn("Path Planner - Place Down - Opening Gripper.")
        self.serv_helper.moveGripper(1)
        rospy.sleep(2)

        # Move down onto block
        rospy.loginfo("Path Planner - Place Down - Lifting up.")
        end_pose.position.z += 0.07
        if not self.move(end_pose, True):
            return False
        
        # Close Gripper
        rospy.loginfo("Path Planner - Place Down - Closing Gripper.")
        self.serv_helper.moveGripper(0)
        rospy.sleep(2)

        return True
