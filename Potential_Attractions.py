# Function for Change in Attractive potential
#calculates potential change from point to goal
#Inputs current position and goal position XYs and distance where laws change. 
# Output is a tuple of the change in potential along x and y axis (deltaX,deltaY)
#Author: Steven Craig
from Distance_Euclidian import *
def PotentialAttractionChange(x,y,xgoal,ygoal,D): #attraction at a specific point
    SF = 0.2 #scaling factor
    d= EuclidianDistance(x,y,xgoal,ygoal)
    if d <= D: #switch to more precise potential at small ranges
        PotentialChange = (SF*x-SF*xgoal,SF*y-SF*ygoal) #in x and y (simplified tuple)
    if d > D:
        PotentialChange = (D*SF*x-D*SF*xgoal/d,D*SF*y-D*SF*ygoal/d)
    return PotentialChange

#Function for Potential attraction
#calculates potential from point to goal
#Inputs current position and goal position XYs and distance where laws change. 
# Ouput is a single value for the Potential at those coordinates.
def PotentialAttraction(x,y,xgoal,ygoal,D): #the attractive field as a whole
    SF = 0.2 #scaling factor
    d = EuclidianDistance(x,y,xgoal,ygoal)
    if d <= D:
        PotentialAtt = 0.5*SF*(d**2)
    else:
        PotentialAtt = D*SF*d - 0.5*SF*D**2
    return PotentialAtt