# Name: Pick Down Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

from PickUp import PickUp

class PickDown(PickUp):
    def __init__(self, ik):
        super().__init__(ik)

    def pick(self, block_name, end_pos):
        pass