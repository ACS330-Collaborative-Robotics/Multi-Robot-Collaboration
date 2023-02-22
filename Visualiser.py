#Author: Steven Craig
#this is the script for getting 3d plots of what the APF sees
import numpy as np
import matplotlib.pyplot as plt
from Potential_Attractions import *
from Potential_Repulsion import *
from APF_Path import *

def Space_Generation(startx,starty,xgoal,ygoal,xobj,yobj,Q,D): #### needs to ad objx and objy
    x = np.linspace(-50, 50, 100)  # Creating X and Y axis
    y = np.linspace(-50, 50, 100)
    X, Y = np.meshgrid(x, y)  # Creates 2 arrays with respective x any y coordination for each point
    PotentialEnergy = np.ndarray(shape=(len(x), len(y)))  # this acts as the z axis on graphs. Works better for visualisation
    for i in range(len(X)):  # gets Z values for the X Y positions
        for j in range(len(Y)):
            PotentialEnergy[i, j] = PotentialAttraction(X[i,j],Y[i,j],xgoal,ygoal,D)
                         # PotentialAttraction(X[i,j],Y[i,j],xgoal,ygoal,D) +PotentialRepulsion(X[i, j], Y[i, j], objx, objy,
    PathTaken = PathPlanner(startx, starty, xgoal, ygoal, xobj, yobj,Q, D)  ## you are here ^^^
    EnergyPathTaken = []
    xline = []
    yline = []
    for i in range(len(PathTaken)):
        x, y = PathTaken[i]
        xline.append(x)
        yline.append(y)
        TotalPotential = PotentialAttraction(x, y, xgoal, ygoal, D) + PotentialRepulsion(x, y, xobj, yobj, Q)
        EnergyPathTaken.append(TotalPotential)
    return X,Y, xline, yline, PotentialEnergy, EnergyPathTaken


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









