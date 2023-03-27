
import rospy

from gazebo_msgs.srv import GetModelState
from block_controller.msg import Blocks
from path_planning.srv import PathPlan
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import tf
import math
from operator import itemgetter

n = 20
layers = math.ceil(n/2)
print(layers)
tower_pos = [] #this has to be a 2 column * layers(value) matrix
h=0
rot=0


for i in range(layers):
    w=0
    home_pos = [w,0,h,0,0,rot]
    for j in range(2):
        home_pos = [w,0,h,0,0,rot]
        tower_pos.append(home_pos)
        w=w+8
    h=h+4

    if rot==0:
        rot=90*(math.pi/180)
    elif rot==90*(math.pi/180):
        rot=0


    
    
height=len(tower_pos)
print(tower_pos)

        