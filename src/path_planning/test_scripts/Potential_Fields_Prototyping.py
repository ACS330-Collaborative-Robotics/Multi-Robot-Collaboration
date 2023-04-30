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
def PotentialAttractionChange(x,y,xgoal,ygoal,D): 
    SF = 0.8 #scaling factor
    d= EuclidianDistance(x,y,xgoal,ygoal)
    if d <= D:
        PotentialChange = (SF*x-SF*xgoal,SF*y-SF*ygoal)
    if d > D:
        PotentialChange = (D*SF*x-D*SF*xgoal/d,D*SF*y-D*SF*ygoal/d)
    return PotentialChange

# Function for EuclidianDistance
# (in the yt video this is the dame as d(q) )
def EuclidianDistance(x,y,xgoal,ygoal):
    d = ((x-xgoal)**2+(y-ygoal)**2)**0.5
    return d

#Function for Potential attraction
#calculates potential from point to goal
#Inputs current position and goal position XYs. Ouput is a single value for the Potential at those coordinates.
def PotentialAttraction(x,y,xgoal,ygoal,D):
    SF = 0.2 #scaling factor
    d = EuclidianDistance(x,y,xgoal,ygoal)
    if d <= D:
        PotentialAtt = 0.5*SF*(d**2)
    else:
        PotentialAtt = D*SF*d - 0.5*SF*D**2
    return PotentialAtt

#Function for the Potential Repulsion
#Inputs current position and obstacle postition XYs. Outputs is a single value for Potentail at those coordinates
def PotentialRepulsion(x,y,xobj,yobj,Q):
    SF = 100000
    d = EuclidianDistance(x,y,xobj,yobj)
    if d <= Q:
        PotentialRep = 0.5*SF*((1/d)-(1/Q))**2
    else:
        PotentialRep = 0
    if PotentialRep > 100:
        PotentialRep = 100
    return PotentialRep

#Function for the Potential Repulsion Change
#Inputs current position and obstacle position XYs. Outputs is 
def PotentialRepulsionChange(x,y,xobj,yobj,Q):
    SF = 20
    d = EuclidianDistance(x,y,xobj,yobj)
    #print(d)
    if d <= Q:
        PotentialRepChangex = -SF*(1/d - 1/Q)*(x-xobj/abs(x-xobj))*1/(d**2)
        PotentialRepChangey = -SF*(1/d - 1/Q)*(y-yobj/abs(y-yobj))*1/(d**2)
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
        diffattx,diffatty = PotentialAttractionChange(PathPointsx[i],PathPointsy[i],xgoal,ygoal,D)
        diffrepx,diffrepy = PotentialRepulsionChange(PathPointsx[i],PathPointsy[i],xobj,yobj,Q)
        difx = diffattx+diffrepx
        dify = diffatty+diffrepy
        d=  EuclidianDistance(x,y,xgoal,ygoal)
        if abs(difx) <0.2 and abs(dify) <0.2 and d > 0.1: # 
            PathComplete = 1
        else:
            #print('Iteration: ',i,'diff x,y: ',diffrepx,diffrepy)
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
startx = 10 # starting points of robot
starty = 50
objx = 10
objy = 10
Q = 10
D = 10
x = np.linspace(-50,50,100) #Creating X and Y axis
y = np.linspace(-50, 50, 100)
X, Y = np.meshgrid(x,y) # Creates 2 arrays with respective x any y coordination for each point
PotentialEnergy = np.ndarray(shape=(len(x),len(y))) #this acts as the z axis on graphs. Works better for visualisation`hu
xgoal = -45 #target position in X and Y
ygoal = -45
for i in range(len(X)): # gets Z values for the X Y positions
    for j in range(len(Y)):
        PotentialEnergy[i,j] = PotentialRepulsion(X[i,j],Y[i,j],objx,objy,Q) # PotentialAttraction(X[i,j],Y[i,j],xgoal,ygoal,D) +
PathTaken = PathPlanner(startx,starty,xgoal,ygoal,objx,objy,Q) ## you are here ^^^
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
    TotalPotential =PotentialAttraction(x,y,xgoal,ygoal,D) + PotentialRepulsion(x,y,objx,objy,Q)
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