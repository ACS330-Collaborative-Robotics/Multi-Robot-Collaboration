#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState

def talker():
    rospy.init_node('block_pos_talker')

    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    rospy.spin()


def callback(data):
    rospy.loginfo(data)

def specific_block_pos():
    # Use service to get position of specific block named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

    print(model_state_service("block1", "world"))

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass