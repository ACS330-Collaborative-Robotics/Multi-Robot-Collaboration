# Name: Pick Up Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

from path_planner import Movement

import rospy
import tf_conversions
from math import pi

class PickUp(Movement.Movement):
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper
    
    def pick(self, block_name):
        """ Pick up specified block

        INPUT: block_name
        OUTPUT: bool Success
        """
        pose = self.serv_helper.getBlockPos(block_name)

        block_orientation_quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        block_orientation_euler = tf_conversions.transformations.euler_from_quaternion(block_orientation_quaternion)

        # Set End Effector orientation to point downwards using quaternions
        orientation_in_euler = [0, pi, block_orientation_euler[2]]
        orientation = tf_conversions.transformations.quaternion_from_euler(orientation_in_euler[0], orientation_in_euler[1], orientation_in_euler[2])
        
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]

        angle_offset = self.serv_helper.fix_block_pose_orientation(pose)
        print(angle_offset)
        if angle_offset == False:
            rospy.logerr("Path Planner - %s - Pick Up - Fix block pose failed.", self.serv_helper.robot_ns)
            return False

        # Open Gripper
        rospy.loginfo("Path Planner - %s - Pick Up - Opening Gripper.", self.serv_helper.robot_ns)
        self.serv_helper.moveGripper(1)
        rospy.sleep(1)

        # Move above block
        rospy.loginfo("Path Planner - %s - Pick Up - Moving above %s.", self.serv_helper.robot_ns, block_name)
        pose.position.z += 0.15
        if not self.move(pose):
            return False
        rospy.sleep(5)
        
        # Move down onto block
        rospy.loginfo("Path Planner - %s - Pick Up - Lowering onto %s.", self.serv_helper.robot_ns, block_name)
        pose.position.z -= 0.05
        if not self.move(pose):
            return False
        
        # Close Gripper
        rospy.loginfo("Path Planner - %s - Pick Up - Closing Gripper.", self.serv_helper.robot_ns)
        self.serv_helper.moveGripper(0)
        rospy.sleep(2)

        # Move down onto block
        rospy.loginfo("Path Planner - %s - Pick Up - Lifting up %s.", self.serv_helper.robot_ns, block_name)
        pose.position.z += 0.10
        if not self.move(pose):
            return False
        
        return True
    
        
