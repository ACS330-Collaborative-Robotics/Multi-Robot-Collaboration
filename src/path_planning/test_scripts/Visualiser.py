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
    xline = PathTaken[0]
    yline = PathTaken[1]
    for i in range(len(PathTaken[0])):

        TotalPotential = PotentialAttraction2d(xline[i], yline[i], xgoal, ygoal, D) + PotentialRepulsion2d(xline[i], yline[i], xobj, yobj, Q)
        EnergyPathTaken.append(TotalPotential)
    #print(EnergyPathTaken)
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
    ax.set_title('X, Y Potential Fields Representation')
    plt.show()
    print("Successfuly run")

def plotPath(PathTaken,xobj,yobj,zobj):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    xpoints = PathTaken[0]
    ypoints = PathTaken[1]
    zpoints = PathTaken[2]

    #for point in PathTaken[0]:
     #   xpoints.append(point[0])
      #  ypoints.append(point[1])
       # zpoints.append(point[2])
    ax.scatter(xobj,yobj,zobj,color='red',linewidth=10)
    ax.plot(xpoints,ypoints,zpoints,linewidth=2.5)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_title('End Effector Path in 3D space')
    plt.show()
    print('PlotPath Complete')









