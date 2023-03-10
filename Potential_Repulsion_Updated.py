#Function for the Potential Repulsion
#Inputs current position and obstacle postition XYs. Outputs is a single value for Potentail at those coordinates
#Author:Steven Craig
from Distance_Euclidian import *
import math
import numpy as np
def PotentialRepulsion(x,y,z,xobj,yobj,zobj,Q):
    SF = 100
    PotentialRep = 0
    for object in range(len(xobj)):
        d = EuclidianDistance(x,y,z,xobj[object],yobj[object],zobj[object])
        if d <= Q[object]:
            PotentialRepcurrent = SF*((1/d)-(1/Q[object]))
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
    for object in range(len(xobj)):
        homevect = [xgoal-x,ygoal-y,zgoal-z]
        objvect = (xobj[object]-x,yobj[object]-y,zobj[object]-z)
        anglegoal = math.atan2(homevect[1],homevect[0])
        angleobj = math.atan2(objvect[1],objvect[0])
        angle = angleobj-anglegoal
        zheight = objvect[2]-homevect[2]
        if angle > 0 or angle == 0:
            repulsionangle = anglegoal - 90
        if angle < 0:
            repulsionangle = anglegoal + 90
        d = EuclidianDistance(x,y,z,xobj[object],yobj[object],zobj[object])
        SF = 5*(d-Q[object])
        repulsionvect = SF*math.cos(angle)*math.cos(repulsionangle),SF*math.cos(angle)*math.sin(repulsionangle)
        if d > Q[object]:
            repulsionvect = 0,0
            zrep = 0
        else:
            zrep = zheight*1/(d-Q[object])
        allvectorsx += repulsionvect[0]
        allvectorsy += repulsionvect[1]
        allvectorsz += zrep
    return allvectorsx,allvectorsy,allvectorsz
