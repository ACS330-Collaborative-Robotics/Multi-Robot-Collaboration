#Function for the Potential Repulsion
#Inputs current position and obstacle postition XYs. Outputs is a single value for Potentail at those coordinates
#Author:Steven Craig
from Distance_Euclidian import *
def PotentialRepulsion(x,y,xobj,yobj,Q): #Repulsive field as a whole
    SF = 100000
    PotentialRep = 0
    for object in range(len(xobj)):
        print(object)
        d = EuclidianDistance(x,y,xobj[object],yobj[object])
        if d <= Q:
            PotentialRepcurrent = 0.5*SF*((1/d)-(1/Q))**2
        else:
            PotentialRepcurrent = 0
        if PotentialRepcurrent > 100:
            PotentialRepcurrent = 100
        PotentialRep += PotentialRepcurrent
    return PotentialRep

#Function for the Potential Repulsion Change
#Inputs current position and obstacle position XYs. Outputs is 
def PotentialRepulsionChange(x,y,xobj,yobj,Q): #repulsion at a specific point
    SF = 20
    #print(d)
    PotentialRepChangex = 0
    PotentialRepChangey = 0
    for object in range(len(xobj)):
        d = EuclidianDistance(x, y, xobj[object], yobj[object])
        if d <= Q: #no repulsion outside of a safe range 
            PotentialRepChangexcurrent = -SF*(1/d - 1/Q)*(x-xobj[object]/abs(x-xobj[object]))*1/(d**2) #'push' in x and y
            PotentialRepChangeycurrent = -SF*(1/d - 1/Q)*(y-yobj(object)/abs(y-yobj(object)))*1/(d**2)
            PotentialRepChangey += PotentialRepChangeycurrent
        else:
            PotentialRepChangex += 0
            PotentialRepChangey += 0
    PotentialRepChange = PotentialRepChangex, PotentialRepChangey #put into tuple
    return PotentialRepChange