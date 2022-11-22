#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from block_controller.msg import Block, Blocks
from math import atan2, asin

def talker():
    rospy.init_node('block_pos_talker')

    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    pub = rospy.Publisher('/blocks_pos', Blocks, queue_size=10)

    rospy.spin()


def callback(data):
    pub = rospy.Publisher('/blocks_pos', Blocks, queue_size=10)

    block_names = []
    for model_name in data.name:
        if "block" in model_name:
            #rospy.loginfo(model_name)
            block_names.append(model_name)

    blocks = []

    for block_num in range(len(block_names)):
        block = Block()

        data = specific_block_pos(block_names[block_num]) # GetModelState
        #rospy.loginfo(data)

        block.block_number = int(block_names[block_num][-1])
        block.x = data.pose.position.x
        block.y = data.pose.position.y
        block.z = data.pose.position.z

        w = data.pose.orientation.w
        x = data.pose.orientation.x
        y = data.pose.orientation.y
        z = data.pose.orientation.z

        block.a = atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z) # Pitch - a
        block.b = atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z) # Roll - b
        block.c = asin(2*x*y + 2*z*w) # Yaw - c

        blocks.append(block)

    pub.publish(blocks)

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