#Function for the Potential Repulsion
#Inputs current position and obstacle postition XYs. Outputs is a single value for Potential at those coordinates
#Author:Steven Craig
from Distance_Euclidian import *
import math
import numpy as np
def PotentialRepulsion(x,y,z,xobj,yobj,zobj,Q):
    SF = 100
    PotentialRep = 0
    for objNum in range(len(xobj)):
        d = EuclidianDistance(x,y,z,xobj[objNum],yobj[objNum],zobj[objNum])
        if d <= Q[objNum]:
            PotentialRepcurrent = SF*((1/d)-(1/Q[objNum]))
        else:
            PotentialRepcurrent = 0
        if PotentialRepcurrent > 100:
            PotentialRepcurrent = 100
        PotentialRep += PotentialRepcurrent
    return PotentialRep

def PotentialRepulsionChange(x,y,z,xobj,yobj,zobj,xgoal,ygoal,zgoal,Q):
    allvectorsx = 0
    allvectorsy = 0
    allvectorsz = 0
    repulsionangle = 0
    for objNum in range(len(xobj)):
        homevect = [xgoal-x,ygoal-y,zgoal-z]
        objvect = (xobj[objNum]-x,yobj[objNum]-y,zobj[objNum]-z)
        anglegoal = math.atan2(homevect[1],homevect[0])
        angleobj = math.atan2(objvect[1],objvect[0])
        angle = angleobj-anglegoal
        zheight = objvect[2]-homevect[2]
        if angle > 0 or angle == 0:
            repulsionangle = anglegoal - 90
        if angle < 0:
            repulsionangle = anglegoal + 90
        d = EuclidianDistance(x,y,z,xobj[objNum],yobj[objNum],zobj[objNum])
        SF = 5*(d-Q[objNum])
        repulsionvect = SF*math.cos(angle)*math.cos(repulsionangle),SF*math.cos(angle)*math.sin(repulsionangle)
        if d > Q[objNum]:
            repulsionvect = 0,0
            zrep = 0
        else:
            zrep = zheight*1/(d-Q[objNum])
        allvectorsx += repulsionvect[0]
        allvectorsy += repulsionvect[1]
        allvectorsz += zrep
    return allvectorsx,allvectorsy,allvectorsz
