# Name: Movement Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

class Movement:
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper
    
    def move(self, pos):
        """ Safely move to desired position using IK, checking robot will stay within zone

        INPUT: Pose pos
        OUTPUT: bool Success - Returns True is movement succesful, False if not possible or failed.
        """
        self.serv_helper.move(pos)

        return True #TODO: Implement zone checks
