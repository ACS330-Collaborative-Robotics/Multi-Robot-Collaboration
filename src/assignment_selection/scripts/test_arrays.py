
import rospy

from gazebo_msgs.srv import GetModelState
from block_controller.msg import Blocks
from path_planning.srv import PathPlan
from std_msgs.msg import String
from geometry_msgs.msg import Pose

import math
from operator import itemgetter

n = 20
layers = math.ceil(n/3)
print(layers)
tower_pos = [] #this has to be a 3 column * layers(value) matrix
h=0
angle=0

for i in range(layers):
    w=0.25
    home_pos = [w,0,h,angle]
    for j in range(3):
        home_pos = [w,0,h,angle]
        tower_pos.append(home_pos)
        w=w+1
    h=h+1

    if angle==0:
        angle=90
    elif angle==90:
        angle=0
    
    

    
    

print(tower_pos)

        