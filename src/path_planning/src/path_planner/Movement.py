# Name: Movement Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
import tf_conversions

from geometry_msgs.msg import Pose

class Movement:
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper
    
    def move(self, pos:Pose):
        """ Safely move to desired position using IK, checking robot will stay within zone

        INPUT: Pose pos
        OUTPUT: bool Success - Returns True is movement succesful, False if not possible or failed.
        """

        # Get coordinates relative to robot instead of world
        pos_robot_frame = self.serv_helper.frameConverter(self.serv_helper.robot_ns, "world", pos)

        # Convert to Euler angles as IK service uses them
        euler_angles = tf_conversions.transformations.euler_from_quaternion([pos_robot_frame.orientation.x, pos_robot_frame.orientation.y, pos_robot_frame.orientation.z, pos_robot_frame.orientation.w])

        pos_robot_frame.orientation.x = euler_angles[0]
        pos_robot_frame.orientation.y = euler_angles[1]
        pos_robot_frame.orientation.z = euler_angles[2]
        pos_robot_frame.orientation.w = 0

        print("Path Planner - Move - Publishing ", self.serv_helper.robot_ns, " to ", pos_robot_frame.position.x, "\t", pos_robot_frame.position.y, "\t", pos_robot_frame.position.z, "\t", pos_robot_frame.orientation.x, "\t", pos_robot_frame.orientation.y, "\t", pos_robot_frame.orientation.z)

        # Move robot to new position, in robot reference frame
        self.serv_helper.move(pos_robot_frame)
        
        #TODO: Force wait until robot has reached desired position. Temp fix:
        rospy.sleep(3)

        return True #TODO: Implement zone checks
