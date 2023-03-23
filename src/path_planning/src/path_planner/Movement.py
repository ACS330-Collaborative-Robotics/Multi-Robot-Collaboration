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

        print("Path Planner - Move - Publishing ", self.serv_helper.robot_ns, " to ", pos_robot_frame.position.x, "\t", pos_robot_frame.position.y, "\t", pos_robot_frame.position.z, "\t", pos_robot_frame.orientation.x, "\t", pos_robot_frame.orientation.y, "\t", pos_robot_frame.orientation.z, "\t", pos_robot_frame.orientation.w)

        # Move robot to new position, in robot reference frame
        status = self.serv_helper.move(pos_robot_frame, final_link_name)

        if not(status):
            print("Path Planner - Error - Target unreachable.")
        
        #TODO: Force wait until robot has reached desired position. Temp fix:
        rospy.sleep(5)

        return status #TODO: Implement zone checks
