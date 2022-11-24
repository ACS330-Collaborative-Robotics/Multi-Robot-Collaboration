#!/usr/bin/env python

# Name: block_slection
# Author: TomRichards (tomtommrichards@gmail.com)

import rospy
import math
from gazebo_msgs.srv import GetModelState
from operator import itemgetter
from std_msgs.msg import String

def choose_block():
    # Define robot namespaces being used - also defines number of robots
    robot_namespaces = ["mover6_a", "mover6_b"]
    rospy.init_node('blockSelector', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    pub = [rospy.Publisher("/mover6_a/nextBlock", String , queue_size=2), rospy.Publisher("/mover6_b/nextBlock", String , queue_size=2)]
    # pub = rospy.Publisher("/mover6_a/nextblock(block1)", std_msgs , queue_size=2)

    # Making array of block names
    blockNames = []
    for i in range(1,21): 
        temp = "block"+str(i)
        blockNames.append(temp)

    # Getting distance from each robot to blocks and sellecting the smallest
    roboColect = []
    for blockName in blockNames:     
        reldist = []
        for robot in robot_namespaces:
            temp =  specific_block_pos(blockName, robot)
            reldist.append(math.sqrt(temp[0]**2+temp[1]**2+temp[2]**2))
        roboColect.append([blockName, reldist.index(min(reldist)), min(reldist)])
    
    # Ordering list on nearest
    sorted(roboColect,key=itemgetter(2))
    
    # Splitting into seperate lists

    goCollect = []
    for i in range(len(robot_namespaces)):
        goCollect.append([])
        for nextBlock in roboColect:
            if nextBlock[1] == i:
                goCollect[i].append(nextBlock[0])
    

    for i in range(max(len(x) for x in goCollect)):
        for j in range(len(pub)):
            try:
                pub[j].publish(goCollect[j][i])
            except:
                pass
        rate.sleep()

    



def specific_block_pos(specific_model_name, reference_model_name):
    # Use service to get position of specific block named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    data = model_state_service(specific_model_name, reference_model_name).pose.position

    # Return ModelState object with position relative to world 
    return [data.x, data.y, data.z]


if __name__ == '__main__':
    try:
        choose_block()
    except rospy.ROSInterruptException:
        pass