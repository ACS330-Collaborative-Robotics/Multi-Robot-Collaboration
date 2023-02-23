## Author: Steven Craig ##
# get distance between points used in APF

# Function for EuclidianDistance
# (in the yt video this is the dame as d(q) )
def EuclidianDistance(x,y,xgoal,ygoal):
    d = ((x-xgoal)**2+(y-ygoal)**2)**0.5 #absolute distance
    return d