from Visualiser import *
#Author: Steven Craig
##inputs-  space is -50 to 50
startx = -5
starty = -5

xgoal = 40
ygoal = 45

xobj = [-20,-30]
yobj = [-40,-30]

Q = 10
D = 5

##Visual Commands
X, Y, xline, yline, PotentialEnergy, EnergyPathTaken = Space_Generation(startx, starty, xgoal, ygoal, xobj, yobj, Q, D)
plotAPF(X, Y, xline, yline, PotentialEnergy, EnergyPathTaken)

##X,Y path the End effector will take
PathTaken = PathPlanner(startx,starty,xgoal, ygoal, xobj, yobj, Q, D)