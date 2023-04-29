# Name: Path Planner Class Definition
# Authors: Conor Nichols (cjnichols1@sheffield.ac.uk) Joseph Fields (jfields1@sheffield.ac.uk)

from path_planner import PickUp, PlaceDown, ServiceHelper, Movement

from geometry_msgs.msg import Pose

import rospy

class PathPlanner:
    def __init__(self, robot_ns:str, block_name:str, end_pos:Pose) -> None:
        # Store init data
        self.robot_ns = robot_ns
        self.target_block = block_name
        self.end_pos = end_pos

        # Setup Service Helper Object
        self.serv_helper = ServiceHelper.ServiceHelper(self.robot_ns,self.target_block)

        # Setup Movement Object
        self.movement = Movement.Movement(self.serv_helper)

        # Setup PickUp/PlaceDown Objects with Service Helper
        self.pickUp = PickUp.PickUp(self.serv_helper)
        self.placeDown = PlaceDown.PlaceDown(self.serv_helper)
        #rospy.loginfo("Path Planner started for %s",self.robot_ns)

    def pathPlan(self) -> bool:
        """ Plan and excute a complete block movement.
        INPUT: robot_ns, block_name, end_pos from object init
        OUTPUT: bool Success
        """
        # Pick up block
        if not self.pickUp.pick(self.target_block):
            return False

        rospy.logfatal("Path Planner - Pick Up Complete.")

        # Put down block
        if not self.placeDown.place(self.end_pos):
            return False
        
        rospy.logfatal("Path Planner - Place Down Complete.")

        return True
    