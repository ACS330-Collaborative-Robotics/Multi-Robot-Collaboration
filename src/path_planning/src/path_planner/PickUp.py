# Name: Pick Up Class Definition
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

class PickUp:
    def __init__(self, serv_helper, movement):
        self.serv_helper = serv_helper
        self.movement = movement
    
    def pick(self, block_name):
        """ Pick up specified block

        INPUT: block_name
        OUTPUT: bool Success
        """
        pose = self.serv_helper.getBlockPos(block_name)

        print("\nGet Pos")
        print(pose)
        
        # Move 5cm above block
        pose.position.z += 0.05

        # Set End Effector orientation
        pose.orientation.x = 0
        pose.orientation.y = 3.14
        pose.orientation.z = 0

        print("\nUpdated pos")
        print(pose)

        if self.movement.move(pose):
            print("Succesfully positioned above block.")

    def moveGripper(self, state):
        pass