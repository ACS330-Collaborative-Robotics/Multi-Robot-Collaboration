#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from block_controller.msg import Block, Blocks

def talker():
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    pub = rospy.Publisher('/blocks_pos', Blocks, queue_size=10)

    rospy.init_node('block_pos_talker')


def callback(data):
    block_names = []
    for model_name in data.name:
        if "block" in model_name:
            rospy.loginfo(model_name)
            block_names.append(model_name)
    rospy.loginfo("\n\n-------------------------------------------")

    blocks = [Block for i in range(len(block_names))]
    for block_num in range(len(block_names)):
        data = specific_block_pos(block_names[block_num]) # GetModelState
        #rospy.loginfo(data)
        blocks[block_num].block_number = int(block_names[block_num][-1])
        blocks[block_num].x = data.pose.position.x
        blocks[block_num].y = data.pose.position.y
        blocks[block_num].z = data.pose.position.z

    rospy.sleep(5)

def specific_block_pos(specific_model_name):
    # Use service to get position of specific block named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

    return model_state_service(specific_model_name, "world")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass