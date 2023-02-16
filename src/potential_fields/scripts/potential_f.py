#! /usr/bin/env python

# Name: Potential Fields Publisher
# Author: Andrei Codin (aacodin1@sheffield.ac.uk)

import numpy as np
import matplotlib.pyplot as plt

##### Overview #####
#Qaudratic potential for an XY position is calculated with the target being global minimum
#This potential is a combination of the attractive for of the end goal and replusive potential from obstacles
#Force is equal to the negative potential gradient.
#This for is used to find joint torques and inverse kinematic for robot.
########################

# Function for Change in Attractive potential
#calculates potential change from point to goal
#Inputs current position and goal position XYs. Output is a tuple of the change in potential along x and y axis (deltaX,deltaY)
def PotentialAttractionChange(x,y,xgoal,ygoal): 
    SF = 1 #scaling factor
    d= EuclidianDistance(x,y,xgoal,ygoal)
    PotentialChange = SF*(x-xgoal,y-ygoal)
    return PotentialChange

# Function for EuclidianDistance
# (in the yt video this is the dame as d(q) )
def EuclidianDistance(x,y,xgoal,ygoal):
    d = ((x-xgoal)**2+(y-ygoal)**2)**0.5
    return d

#Function for Potential attraction
#calculates potential from point to goal
#Inputs current position and goal position XYs. Ouput is a single value for the Potential at those coordinates.
def PotentialAttraction(x,y,xgoal,ygoal):
    SF = 0.2 #scaling factor
    d = EuclidianDistance(x,y,xgoal,ygoal)
    PotentialAtt = 0.5*SF*d**2
    return PotentialAtt

#Function for the Potential Repulsion
#Inputs current position and obstacle postition XYs. Outputs is a single value for Potentail at those coordinates
def PotentialRepulsion(x,y,xobj,yobj,Q):
    SF = 20
    d = EuclidianDistance(x,y,xobj,yobj)
    if d <= Q:
        PotentialRep = 0.54*SF*((1/d)-(1/Q))**2
    else:
        PotentialRep = 0
    return PotentialRep

#Function for the Potential Repulsion Change
#Inputs current position and obstacle position XYs. Outputs is 
def PotentialRepulsionChange(x,y,xobj,yobj,Q):
    SF = 20
    d = EuclidianDistance(x,y,xobj,yobj)
    if d <= Q:
        PotentialRepChangex = ((1/Q)-(1/(x-xobj)))*SF*(1/d**2)
        PotentialRepChangey = ((1/Q)-(1/(y-yobj)))*SF*(1/d**2)
        PotentialRepChange = PotentialRepChangex,PotentialRepChangey
    else:
        PotentialRepChange = (0,0)
    return PotentialRepChange


#Function for viapoints on the path
#returned as an array of points
#Takes the start position and goal position XYs. Output an array of the via points ((x1,y1),(x2,y2),(x3,y3)....)
def PathPlanner(x,y,xgoal,ygoal,xobj,yobj,Q): #you are currently trying to add this in, this is the path from a poiint using position and force ads velocity
    PathComplete = 0 #This turns to 1 and ends the function once end effector has reached target position (minimum of pootential)
    PathPointsx = [x] #First X and Y points
    PathPointsy = [y] #These are in different arrays cos tuples suck. The 'zip' function at the end turns them into a tuple
    i = 0
    while PathComplete == 0:
        diffattx,diffatty = PotentialAttractionChange(PathPointsx[i],PathPointsy[i],xgoal,ygoal)
        diffrepx,diffrepy = PotentialRepulsionChange(PathPointsx[i],PathPointsy[i],xobj,yobj,Q)
        difx = diffattx+diffrepx
        dify = diffatty+diffrepy
        if abs(difx) <0.2 and abs(dify) <0.2: # 
            PathComplete = 1
        else:
            print('Iteration: ',i,'diff x,y: ',difx,dify)
            nextx = PathPointsx[i] - difx*0.2
            nexty = PathPointsy[i] - dify*0.2
            x = nextx
            y = nexty
            PathPointsx.append(x)
            PathPointsy.append(y)
        i += 1
    PathPoints = list(zip(PathPointsx,PathPointsy))
    return PathPoints

        

# Generating axis values
startx = 15 # starting points of robot
starty = 20
objx = 10
objy = 10
Q = 10
x = np.linspace(-20,20,40) #Creating X and Y axis
y = np.linspace(-20, 20, 40)
X, Y = np.meshgrid(x,y) # Creates 2 arrays with respective x any y coordination for each point
PotentialEnergy = np.ndarray(shape=(len(x),len(y))) #this acts as the z axis on graphs. Works better for visualisation
xgoal = 2 #target position in X and Y
ygoal = 2
for i in range(len(X)): # gets Z values for the X Y positions
    for j in range(len(Y)):
        PotentialEnergy[i,j] = PotentialAttraction(X[i,j],Y[i,j],xgoal,ygoal) + PotentialRepulsion(X[i,j],Y[i,j],objx,objy,Q)
PathTaken = PathPlanner(startx,starty,xgoal,ygoal,objx,objy,Q)
#print(PathTaken)
#energychange = PotentialAttractionChange(2,2,2,2)
#print(energychange)
EnergyPathTaken = []
xline =[]
yline =[]
for i in range(len(PathTaken)):
    x,y = PathTaken[i]
    xline.append(x)
    yline.append(y)
    TotalPotential =PotentialAttraction(x,y,xgoal,ygoal) + PotentialRepulsion(x,y,objx,objy,Q)
    EnergyPathTaken.append(TotalPotential)


# Making 3d Plot
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot_surface(X,Y,PotentialEnergy)
ax.plot(xline,yline,EnergyPathTaken,color='red',linewidth=4.5)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
plt.show()
print("Successfuly run")