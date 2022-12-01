# Name: Pick Up Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

class PickUp:
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper
    
    def pick(self, block_name):
        pose = self.serv_helper.getBlockPos(block_name)

        print(pose)
        
        # Move 5cm above block
        pose.position.z += 0.05

        # Set End Effector orientation
        pose.orientation.x = 0
        pose.orientation.y = 3.14
        pose.orientation.z = 0

        print(pose)

        self.position(pose)

    def position(self, pos):
        self.serv_helper.move(pos)

    def moveGripper(self, state):
        pass