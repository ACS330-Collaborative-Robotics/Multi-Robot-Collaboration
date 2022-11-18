import random

# blocks = 10
# robots = 2
# decimalplace = 2

def randLocals(blocks,robots):
    # This files generates random locations for the blocks and the robots and puts them in a list of list
    # its inputs are the number of blocks
    # the number of robots

    output = []

    for i in range(blocks+robots):
        output.append([random.randint(0,999)/10**2,random.randint(0,999)/10**2])
    return output