# Name: Movement Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy

from geometry_msgs.msg import Pose

class Movement:
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper
    
    def move(self, pos:Pose, final_link_name=""):
        """ Safely move to desired position using IK, checking robot will stay within zone

        INPUT: Pose pos
        OUTPUT: bool Success - Returns True is movement succesful, False if not possible or failed.
        """

        # Get coordinates relative to robot instead of world
        pos_robot_frame = self.serv_helper.frameConverter(self.serv_helper.robot_ns, "world", pos)

        rospy.loginfo("Path Planner - Move - Publishing %s to\t%.2f\t%.2f\t%.2f\t\t%.2f\t%.2f\t%.2f\t%.2f", self.serv_helper.robot_ns, pos_robot_frame.position.x, pos_robot_frame.position.y, pos_robot_frame.position.z, pos_robot_frame.orientation.x, pos_robot_frame.orientation.y, pos_robot_frame.orientation.z, pos_robot_frame.orientation.w)

        # Move robot to new position, in robot reference frame
        status = self.serv_helper.move(pos_robot_frame, final_link_name)

        if not(status):
            rospy.logerr("Path Planner - Error, Target position unreachable.")
        else :
            #TODO: Force wait until robot has reached desired position. Temp fix:
            rospy.sleep(8)

        return status #TODO: Implement zone checks
