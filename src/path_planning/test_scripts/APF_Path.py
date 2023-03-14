import numpy as np
from Potential_Attractions import *
from Potential_Repulsion_Updated import *
from Distance_Euclidian import *
#Author: Steven Craig
#Function for viapoints on the path
#returned as an array of points
#Takes the start position and goal position XYs. Output an array of the via points ((x1,y1),(x2,y2),(x3,y3)....)
#xobj is an array of all obstacle x points, y is the same but for y points
def PathPlanner(x,y,z,xgoal,ygoal,zgoal,xobj,yobj,zobj,Q,D): #you are currently trying to add this in, this is the path from a poiint using position and force ads velocity
    PathComplete = 0 #This turns to 1 and ends the function once end effector has reached target position (minimum of pootential)
    PathPointsx = [x] #First X and Y points
    PathPointsy = [y] #These are in different arrays cos tuples suck. The 'zip' function at the end turns them into a tuple
    PathPointsz = [z]
    i = 0
    while PathComplete == 0:
        diffatt = PotentialAttractionChange(PathPointsx[i],PathPointsy[i],PathPointsz[i],xgoal,ygoal,zgoal,D)
        diffrep = PotentialRepulsionChange(PathPointsx[i],PathPointsy[i],PathPointsz[i],xobj,yobj,zobj,xgoal,ygoal,zgoal,Q)
        difx = diffatt[0] + diffrep[0]
        dify = diffatt[1] + diffrep[1]
        difz = diffatt[2] + diffrep[2]
        d = EuclidianDistance(x,y,z,xgoal,ygoal,zgoal)
        if abs(difx) <0.2 and abs(dify) <0.2 and abs(difz) <0.2 and d < 2:#
            PathComplete = 1
        if abs(difx) < 0.1 and abs(dify) < 0.1:
            pass
            print("LOCAL MINIMA")
            #add get out of minima here
        else:
            #print('Iteration: ',i,'x,y: ',PathPointsx,PathPointsy)
            nextx = PathPointsx[i] - 1.5*difx
            nexty = PathPointsy[i] - 1.5*dify
            nextz = PathPointsz[i] - 1.5*difz
            x = nextx
            y = nexty
            z = nextz
            PathPointsx.append(x)
            PathPointsy.append(y)
            PathPointsz.append(z)
        i += 1
        #print(PathPointsx[i],PathPointsy[i])
    #PathPoints = list(zip(PathPointsx,PathPointsy))
    print('Path Complete')
    print(len(PathPointsx))
    return PathPointsx,PathPointsy,PathPointsz