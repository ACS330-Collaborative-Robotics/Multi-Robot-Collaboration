#!/usr/bin/env python

# Name: block_slection
# Author: TomRichards (tomtommrichards@gmail.com)

import rospy

def choose_block():
    # Define robot namespaces being used - also defines number of robots
    robot_namespaces = ["mover6_a", "mover6_b"]

    # pub = rospy.Publisher("/mover6_a/nextblock(block1)", std_msgs , queue_size=2)

    #Making array of block names
    blockNames = []
    for i in range(1,21):
        temp = "block"+str(i)
        blockNames.append(temp)

    print(blockNames)

    



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