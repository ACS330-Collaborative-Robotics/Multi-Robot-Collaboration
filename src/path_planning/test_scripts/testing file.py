import numpy as np
import math
def PotentialAttractionChange(x,y,xgoal,ygoal): 
    SF = 1 #scaling factor
    d= EuclidianDistance(x,y,xgoal,ygoal)
    PotentialChange = SF*(x-xgoal,y-ygoal)
    return PotentialChange
def EuclidianDistance(x,y,xgoal,ygoal):
    d = ((x-xgoal)**2+(y-ygoal)**2)**0.5
    return d

def PathPlanner(x,y,xgoal,ygoal): #you are currently trying to add this in, this is the path from a poiint using position and force ads velocity
    PathComplete = 0 #This turns to 1 and ends the function once end effector has reached target position (minimum of pootential)
    PathPointsx = [x] #First X and Y points
    PathPointsy = [y] #These are in different arrays cos tuples suck. The 'zip' function at the end turns them into a tuple
    i = 0
    while PathComplete == 0:
        difx,dify = PotentialAttractionChange(PathPointsx[i],PathPointsy[i],xgoal,ygoal)
        if abs(difx) <0.2 and abs(dify) <0.2: # 
            PathComplete = 1
        else:
            #print('Iteration: ',i,'diff x,y: ',difx,dify)
            nextx = PathPointsx[i] - difx*0.2
            nexty = PathPointsy[i] - dify*0.2
            x = nextx
            y = nexty
            PathPointsx.append(x)
            PathPointsy.append(y)
        i += 1
    PathPoints = list(zip(PathPointsx,PathPointsy))
    return PathPoints



TestArr = PathPlanner(-10,-10,15,15)
xpoints =[]
ypoints = []
for point in TestArr:
    xpoints.append(point[0])
    ypoints.append(point[1])
currentx, currenty = 3,1
xobj, yobj = -7,-5
xgoal,ygoal = -5,-2
anglegoal = math.atan2(ygoal-currenty,xgoal-currentx)
angleobj = math.atan2(yobj-currenty,xobj-currentx)
print(math.degrees(angleobj-anglegoal))

testvec = [1,2]
rotate90 = [[0,1],
            [-1,0]]
rotatemin90 = [[0 -1],
             [1, 0]]
ans = np.dot(rotate90,testvec)
print(ans)