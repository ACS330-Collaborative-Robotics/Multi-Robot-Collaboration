from Visualiser import *
from Object_Creation import *
#Author: Steven Craig
##inputs-  space is -50 to 50
startx = -50
starty = -50
startz = 50
xgoal = 40
ygoal = 40
zgoal = 0
xobj = [0,10,-2]
yobj = [-40,-10,15]
zobj = [20,15,10]
Q = [12,4,2]
D = 5
xobj,yobj,zobj,Q = Link_Midpoints(xobj,yobj,zobj,Q)
##Visual Commands
X,Y,xline, yline, PotentialEnergy, EnergyPathTaken, PathTaken = Space_Generation(startx, starty,startz,xgoal, ygoal,zgoal, xobj, yobj,zobj, Q, D)
plotAPF(X, Y, xline, yline, PotentialEnergy, EnergyPathTaken)
plotPath(PathTaken)
##X,Y path the End effector will take
PathTakenx, PathTakeny, PathTakenz = PathPlanner(startx,starty,startz,xgoal, ygoal,zgoal, xobj, yobj,zobj, Q, D)

#print(PathTakenx)
#print(PathTakeny)
#print(PathTakenz)
