#Author: Steven Craig
#The aim of this file its to take in the joint positions.
#From this it will generate a more detailed model of the objstactle to give to apth.
#The output with be a larger list of objstacle positions
import math
from Distance_Euclidian import *
from Visualiser import *
def Link_Midpoints(xobj,yobj,zobj,Q): ####you are here
    
    no_links = len(xobj) -1
    newxobj = []
    newyobj = []
    newzobj = []
    newQ = []
    if no_links ==0:
        newxobj = xobj
        newyobj = yobj
        newzobj = zobj
        newQ = Q
    else:
        for i in range(no_links):
            vector = [xobj[i + 1] - xobj[i], yobj[i + 1] - yobj[i], zobj[i + 1] - zobj[i]]
            detlaQ = Q[i+1]-Q[i]
            for j in range(20):
                newxobj.append(xobj[i] + vector[0]*j/20)
                newyobj.append(yobj[i] + vector[1] * j / 20)
                newzobj.append(zobj[i] + vector[2] * j / 20)
                newQ.append(Q[i] + detlaQ*j/10)

    #for i in range(no_links):
     #   vector = [xobj[i+1]-xobj[i],yobj[i+1]-yobj[i],zobj[i+1]-zobj[i]]
      #  d = EuclidianDistance(xobj[i],yobj[i],zobj[i],xobj[i+1],yobj[i+1],zobj[i+1])
       # zangle = math.asin(vector[2]/d)
        #angle = math.atan2(vector[0],vector[1])
        #tangentangle = angle + 90
        #ztangent = zangle +90
        #newvector = math.cos(tangentangle),math.sin(tangentangle)
        #zvect = math.cos(ztangent)
        #J0 = xobj[i] + Q[i]*newvector[0],yobj[i] + Q[i]*newvector[1]
        #J1 = xobj[i+1] + Q[i+1]*newvector[0],yobj[i+1] + Q[i+1]*newvector[1]
        #K0 = xobj[i] - Q[i]*newvector[0],yobj[i] - Q[i]*newvector[1]
        #K1 = xobj[i+1] - Q[i+1]*newvector[0],yobj[i+1] - Q[i+1]*newvector[1]
        #Link1 = J1[0]-J0[0],J1[1]-J0[1]
        #Link2 = K1[0]-K0[0],K1[1]-K0[1]
        #j = 0
        #for j in range(8):
        #    if j == 0:
        #        newxobj.append(xobj[i])
         #       newyobj.append(yobj[i])
         #       newQ.append(Q[i])
         #   else:
         #       newxobj.append(xobj[i]+(vector[0]*j)/8)
         #       newyobj.append(yobj[i] + (vector[1] * j) / 8)
         #       stepL1 = J0[0]+Link1[0],J0[1]+Link1[1]
         #       stepL2 = J0[0]+Link2[0],J0[1]+Link2[1]
         #       d = EuclidianDistance(stepL1[0],stepL1[1],stepL2[0],stepL2[1])
         #       newQ.append(d)
        #plotPath(Link1)
        #plotPath(Link2)
        #
    print('Midpoints Complete')
    return newxobj,newyobj,newzobj,newQ