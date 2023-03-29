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

        block_orientation_in_quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        block_orientation_in_euler = tf_conversions.transformations.euler_from_quaternion(block_orientation_in_quaternion)

        rospy.logwarn("Path Planner - Pick Up -\t%.2f\t%.2f\t%.2f", block_orientation_in_quaternion[0]*180/pi, block_orientation_in_quaternion[1]*180/pi, block_orientation_in_quaternion[2]*180/pi)

        # Move 5cm above block
        pose.position.z += 0.25

        # Set End Effector orientation to point downwards using quaternions
        orientation_in_euler = [0,180*pi/180, 0]
        orientation_quaternion = tf_conversions.transformations.quaternion_from_euler(orientation_in_euler[0], orientation_in_euler[1], orientation_in_euler[2])
        
        pose.orientation.x = orientation_quaternion[0]
        pose.orientation.y = orientation_quaternion[1]
        pose.orientation.z = orientation_quaternion[2]
        pose.orientation.w = orientation_quaternion[3]
        rospy.loginfo("Path Planner - Pick Up - Moving to %s", block_name)
        
        return self.move(pose)

    def moveGripper(self, state):
        pass