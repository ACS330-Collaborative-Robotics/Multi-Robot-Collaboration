#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import matlab.engine


def selection_prossess():
    # https://uk.mathworks.com/help/matlab/matlab_external/call-matlab-functions-from-python.html
    # For each block the code should know [[Name/Number],[x,y],[a,b,c]]

    # This will take the inputs:
    # All Blocks Locations
    # How many Arms
    # Arms locations/type/reach
    # surching algotithum requested by user

    # This should then output:
    # [Who - what robot, What - Which block, Where - Start and end position of each block, When - order and timeings]

    # Setting Up the MATLAB fucntions
    eng = matlab.engine.start_matlab()
    eng.cd(r'myFolder', nargout=0) # Change this to where the file is stored
    # outArray = eng.SelectionProcess()
    # outArray = [distRobo1,Brick ,DistRobo2,Brick]

    # Reciving the Block locations
    # Format [[x1,y1],[x2,y2],...]


if __name__ == '__main__':
    try:
        selection_prossess()
    except rospy.ROSInterruptException:
        pass