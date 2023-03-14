#Author: Steven Craig
#this is the script for getting 3d plots of what the APF sees
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from APF_Path import *

def Space_Generation(startx,starty,startz,xgoal,ygoal,zgoal,xobj,yobj,zobj,Q,D): #### needs to ad objx and objy
    x = np.linspace(-50, 50, 100)  # Creating X and Y axis
    y = np.linspace(-50, 50, 100)
    X, Y = np.meshgrid(x, y)  # Creates 2 arrays with respective x any y coordination for each point
    PotentialEnergy = np.ndarray(shape=(len(x), len(y)))  # this acts as the z axis on graphs. Works better for visualisation
    for i in range(len(X)):  # gets Z values for the X Y positions
        for j in range(len(Y)):
            PotentialEnergy[i, j] = PotentialAttraction2d(X[i,j],Y[i,j],xgoal,ygoal,D)+ PotentialRepulsion2d(X[i,j],Y[i,j],xobj,yobj,Q)
                         # PotentialAttraction(X[i,j],Y[i,j],xgoal,ygoal,D) +PotentialRepulsion(X[i, j], Y[i, j], objx, objy,
    PathTaken = PathPlanner(startx, starty,startz, xgoal, ygoal,zgoal, xobj, yobj,zobj,Q, D)  ## you are here ^^^
    EnergyPathTaken = []
    xline = []
    yline = []
    zline = []
    for i in range(len(PathTaken)):
        xp = PathTaken[0]
        yp = PathTaken[1]
        #zp =PathTaken[2]
        xline.append(xp[i])
        yline.append(yp[i])
        #zline.append(zp[i])
        TotalPotential = PotentialAttraction2d(xp[i], yp[i], xgoal, ygoal, D) + PotentialRepulsion2d(xp[i], yp[i], xobj, yobj, Q)
        EnergyPathTaken.append(TotalPotential)
    print('Space Generation Complete')
    return X,Y,xline, yline, PotentialEnergy, EnergyPathTaken, PathTaken


def plotAPF(X,Y, xline, yline, PotentialEnergy,EnergyPathTaken):
    # Making 3d Plot
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot_surface(X, Y, PotentialEnergy)
    ax.plot(xline, yline, EnergyPathTaken, color='red', linewidth=4.5)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    plt.show()
    print("Successfuly run")

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









