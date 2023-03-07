#Author: Steven Craig
#The aim of this file its to take in the joint positions.
#From this it will generate a more detailed model of the objstactle to give to apth.
#The output with be a larger list of objstacle positions
import math
from Distance_Euclidian import *
def Link_Midpoints(xobj,yobj,Q):
    no_links = len(xobj) -1
    newxobj = []
    newyobj = []
    newQ = []
    for i in range(no_links):
        vector = xobj[i+1]-xobj[i],yobj[i+1]-yobj[i]
        angle = math.atan2(vector[0],vector[1])
        tangentangle = angle + 90
        newvector = math.cos(tangentangle),math.sin(tangentangle)
        J0 = xobj[i] + Q[i]*newvector[0],yobj[i] + Q[i]*newvector[1]
        J1 = xobj[i+1] + Q[i+1]*newvector[0],yobj[i+1] + Q[i+1]*newvector[1]
        K0 = xobj[i] - Q[i]*newvector[0],yobj[i] - Q[i]*newvector[1]
        K1 = xobj[i+1] - Q[i+1]*newvector[0],yobj[i+1] - Q[i+1]*newvector[1]
        Link1 = K0[0]-J0[0],K0[1]-J0[1]
        Link2 = K1[0]-J1[0],K1[1]-J1[1]
        j = 0
        for j in range(3):
            if j == 0:
                newxobj.append(xobj[i])
                newyobj.append(yobj[i])
                newQ.append(Q[i])
            else:
                newxobj.append(xobj[i]+(vector[0]*i)/4)
                newxobj.append(xobj[i] + (vector[0] * i) / 4)
                stepL1 = J0[0]+Link1[0],J0[1]+Link1[1]
                stepL2 = J0[0]+Link2[0],J0[1]+Link2[1]
                d = EuclidianDistance(stepL1[0],stepL1[1],stepL2[0],stepL2[1])
                newQ.append(d)
    return newxobj,newyobj,newQ