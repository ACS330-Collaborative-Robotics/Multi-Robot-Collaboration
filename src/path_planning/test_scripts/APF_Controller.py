from Visualiser import *
from Object_Creation import *
#Author: Steven Craig
##inputs-  space is -50 to 50
startx = 45
starty = 35
startz = 20
xgoal = -15
ygoal = -45
zgoal = -40
xobj = [40,30,15,0,-15,-15,-10,-5,0]
yobj = [25,10,5,0,-15,-20,-20,-20,-25]
zobj = [15,10,5,2,0,-25,-30,-35,-35]
Q = [30,30,30,30,30,30,30,30,30]
D = 5
#xobj,yobj,zobj,Q = Link_Midpoints(xobj,yobj,zobj,Q)
##Visual Commands
X,Y,xline, yline, PotentialEnergy, EnergyPathTaken, PathTaken = Space_Generation(startx, starty,startz,xgoal, ygoal,zgoal, xobj, yobj,zobj, Q, D)
plotAPF(X, Y, xline, yline, PotentialEnergy, EnergyPathTaken)
plotPath(PathTaken,xobj,yobj,zobj )
##X,Y path the End effector will take
#PathTakenx, PathTakeny, PathTakenz = PathPlanner(startx,starty,startz,xgoal, ygoal,zgoal, xobj, yobj,zobj, Q, D)

#print(PathTakenx)
#print(PathTakeny)
#print(PathTakenz)
