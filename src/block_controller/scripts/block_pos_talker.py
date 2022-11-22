#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from pprint import pprint
from block_controller.msg import Block, Blocks

def talker():
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    pub = rospy.Publisher('/blocks_pos', Blocks, queue_size=10)

    rospy.init_node('block_pos_talker')


def callback(data):
    len = 0;
    for model_name in data.name:
        if "block" in model_name:
            rospy.loginfo(model_name)
            rospy.loginfo(specific_block_pos(model_name))
            len += 1
    rospy.loginfo("\n\n-------------------------------------------")

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