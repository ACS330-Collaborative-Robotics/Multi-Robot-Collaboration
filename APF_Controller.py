from Visualiser import *
#Author: Steven Craig
##inputs-  space is -50 to 50
startx = -50
starty = -50

xgoal = 40
ygoal = 40

xobj = [-20,10]
yobj = [-15,0]

Q = 12
D = 5

##Visual Commands
X, Y, xline, yline, PotentialEnergy, EnergyPathTaken, PathTaken = Space_Generation(startx, starty, xgoal, ygoal, xobj, yobj, Q, D)
plotAPF(X, Y, xline, yline, PotentialEnergy, EnergyPathTaken)
plotPath(PathTaken)
##X,Y path the End effector will take
PathTaken = PathPlanner(startx,starty,xgoal, ygoal, xobj, yobj, Q, D)
print(len(PathTaken))