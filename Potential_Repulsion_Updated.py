#Function for the Potential Repulsion
#Inputs current position and obstacle postition XYs. Outputs is a single value for Potentail at those coordinates
#Author:Steven Craig
from Distance_Euclidian import *
import math
import numpy as np
def PotentialRepulsion(x,y,xobj,yobj,Q):
    SF = 5000
    PotentialRep = 0
    for object in range(len(xobj)):
        d = EuclidianDistance(x,y,xobj[object],yobj[object])
        if d <= Q:
            PotentialRepcurrent = SF*((1/d)-(1/Q))
        else:
            PotentialRepcurrent = 0
        if PotentialRepcurrent > 100:
            PotentialRepcurrent = 100
        PotentialRep += PotentialRepcurrent
    return PotentialRep

def PotentialRepulsionChange(x,y,xobj,yobj,xgoal,ygoal,Q):
    allvectorsx = 0
    allvectorsy = 0
    repulsionangle = 0
    for object in range(len(xobj)):

        homevect = (xgoal-x,ygoal-y)
        objvect = (xobj[object]-x,yobj[object]-y)
        anglegoal = math.atan2(homevect[1],homevect[0])
        angleobj = math.atan2(objvect[1],objvect[0])
        angle = angleobj-anglegoal
        if angle > 0 or angle == 0:
            repulsionangle = anglegoal - 90
        if angle < 0:
            repulsionangle = anglegoal + 90
        d = EuclidianDistance(x,y,xobj[object],yobj[object])
        SF = 5*(d-Q)
        repulsionvect = SF*math.cos(angle)*math.cos(repulsionangle),SF*math.cos(angle)*math.sin(repulsionangle)
        if d > Q:
            repulsionvect = 0,0
        allvectorsx += repulsionvect[0]
        allvectorsy += repulsionvect[1]
    return allvectorsx,allvectorsy
