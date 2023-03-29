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
        
        # Move 5cm above block
        end_pose.position.z += 0.25

        # Set End Effector orientation to point downwards using quaternions
        orientation_in_euler = [0,180*pi/180,0]
        orientation = tf_conversions.transformations.quaternion_from_euler(orientation_in_euler[0], orientation_in_euler[1], orientation_in_euler[2])
        
        end_pose.orientation.x = orientation[0]
        end_pose.orientation.y = orientation[1]
        end_pose.orientation.z = orientation[2]
        end_pose.orientation.w = orientation[3]
        rospy.loginfo("Path Planner - Place down - Moving to %.2f\t%.2f\t%.2f", end_pose.position.x, end_pose.position.y, end_pose.position.z)
        
        return self.move(end_pose)