# Name: Pick Down Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

from path_planner import PickUp

class PickDown(PickUp.PickUp):
    def __init__(self, serv_helper):
        super().__init__(serv_helper)

    def pick(self, block_name, end_pos):
        pass