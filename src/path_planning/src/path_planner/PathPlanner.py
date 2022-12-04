# Name: Path Planner Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

from path_planner import PickUp, PickDown, ServiceHelper, Movement

from geometry_msgs.msg import Pose

class PathPlanner:
    def __init__(self, robot_ns:str, block_name:str, end_pos:Pose) -> None:
        # Store init data
        self.robot_ns = robot_ns
        self.target_block = block_name
        self.end_pos = end_pos

        # Setup Inverse Kinematics Object
        self.serv_helper = ServiceHelper.ServiceHelper(self.robot_ns)

        # Setup Movement Object
        self.movement = Movement.Movement(self.serv_helper)

        # Setup PickUp/Down Objects with Service Helper and Movement
        self.pickUp = PickUp.PickUp(self.serv_helper, self.movement)
        self.pickDown = PickDown.PickDown(self.serv_helper, self.movement)


    def pathPlan(self) -> bool:
        """ Plan and excute a complete block movement.

        INPUT: robot_ns, block_name, end_pos from object init
        OUTPUT: bool Success
        """
        # Pick up block
        #self.pickUp.pick(self.target_block)

        # Move arm
        #TODO: Convert from world frame to robot frame
        print("Finding transform")
        end_pos_robot_frame = self.serv_helper.frameConverter(self.robot_ns, "world", self.end_pos)

        self.movement.move(end_pos_robot_frame)

        # Put down block
        #self.pickDown.pick(self.target_block, self.end_pos)

        return True
    