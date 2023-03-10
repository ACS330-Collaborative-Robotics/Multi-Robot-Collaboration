#Author: Steven Craig
#this is the script for getting 3d plots of what the APF sees
import numpy as np
import matplotlib.pyplot as plt
from APF_Path import *

def Space_Generation(startx,starty,startz,xgoal,ygoal,zgoal,xobj,yobj,zobj,Q,D): #### needs to ad objx and objy

    PathTaken = PathPlanner(startx, starty,startz, xgoal, ygoal,zgoal, xobj, yobj,zobj,Q, D)  ## you are here ^^^
    EnergyPathTaken = []
    xline = []
    yline = []
    zline = []
    for i in range(len(PathTaken)):
        xp = PathTaken[0]
        yp = PathTaken[1]
        zp =PathTaken[2]
        xline.append(xp[i])
        yline.append(yp[i])
        zline.append(zp[i])
        TotalPotential = PotentialAttraction(xp, yp, zp, xgoal, ygoal, zgoal, D) + PotentialRepulsion(xp, yp,zp, xobj, yobj,zobj, Q)
        EnergyPathTaken.append(TotalPotential)
    print('Space Generation Complete')
    return xline, yline,zline, EnergyPathTaken, PathTaken


def plotAPF(X,Y,Z, xline, yline,zline, PotentialEnergy,EnergyPathTaken):
    # Making 3d Plot
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot_surface(X, Y, Z,PotentialEnergy)
    ax.plot(xline, yline, zline,EnergyPathTaken, color='red', linewidth=4.5)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    plt.show()
    print("PlotAPF complete")

def plotPath(PathTaken):
    fig = plt.figure()
    ax = plt.axes()
    xpoints =[]
    ypoints = []
    zpoints = []
    for point in PathTaken:
        xpoints.append(point[0])
        ypoints.append(point[1])
        zpoints.append(point[2])
    ax.plot(xpoints,ypoints,zpoints)
    plt.show()
    print('PlotPath Complete')









