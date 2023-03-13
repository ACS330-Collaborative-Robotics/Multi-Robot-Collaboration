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
#print(xobj,yobj,Q)
##Visual Commands
X, Y,Z, xline, yline,zline, PotentialEnergy, EnergyPathTaken, PathTaken = Space_Generation(startx, starty,startz,xgoal, ygoal,zgoal, xobj, yobj,zobj, Q, D)
plotAPF(xobj, yobj,zobj, xline, yline,zline, PotentialEnergy, EnergyPathTaken)
#plotPath(PathTaken)
##X,Y path the End effector will take
PathTaken = PathPlanner(startx,starty,startz,xgoal, ygoal,zgoal, xobj, yobj,zobj, Q, D)
#print(len(PathTaken))