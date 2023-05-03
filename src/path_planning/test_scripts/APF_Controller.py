from Visualiser import *
from Object_Creation import *
#Author: Steven Craig
##inputs-  space is -50 to 50
startx = 5
starty = 25
startz = 20
xgoal = 0
ygoal = -25
zgoal = -20
xobj = [0,0,0,0,0]
yobj = [0,0,0,0,0]
zobj = [0,5,10,20,50]
Q = [7,16,16,20,30]
D = 5
xobj,yobj,zobj,Q = Link_Midpoints(xobj,yobj,zobj,Q)
##Visual Commands
X,Y,xline, yline, PotentialEnergy, EnergyPathTaken, PathTaken = Space_Generation(startx, starty,startz,xgoal, ygoal,zgoal, xobj, yobj,zobj, Q, D)
plotAPF(X, Y, xline, yline, PotentialEnergy, EnergyPathTaken)
plotPath(PathTaken,xobj,yobj,zobj )
##X,Y path the End effector will take
#PathTakenx, PathTakeny, PathTakenz = PathPlanner(startx,starty,startz,xgoal, ygoal,zgoal, xobj, yobj,zobj, Q, D)

#print(PathTakenx)
#print(PathTakeny)
#print(PathTakenz)
