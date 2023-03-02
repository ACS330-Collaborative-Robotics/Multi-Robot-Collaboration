#!/usr/bin/env python

# Name: Block Position Publisher from Gazebo Model States
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from block_controller.msg import Block, Blocks
from math import atan2, asin

def talker():
    # Setup Node
    rospy.init_node('block_pos_talker')

    # Setup subscriber callback - Object ModelStates
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    # Setup publisher - Object Blocks
    pub = rospy.Publisher('/blocks_pos', Blocks, queue_size=10)

    # Waits until node is shutdown or subscriber callback occurs
    rospy.spin()


def callback(data):
    # Setup publisher in function
    pub = rospy.Publisher('/blocks_pos', Blocks, queue_size=10)

    # Set rate to publish block positions at
    publish_rate = 1

    # Use subscriber method to get a list of all the block names
    block_names = []
    for model_name in data.name:
        if "block" in model_name:
            #rospy.loginfo(model_name)
            block_names.append(model_name)

    # Empty array for Block() objects
    blocks = []

    # Iterate through all blocks
    for block_num in range(len(block_names)):
        block = Block() # Empty Block object to fill

        # Get data for specific
        data = specific_block_pos(block_names[block_num]) # GetModelState
        #rospy.loginfo(data)

        block.block_number = int(block_names[block_num].replace("block", "")) # Block number

        # Position x y z
        block.x = data.pose.position.x
        block.y = data.pose.position.y
        block.z = data.pose.position.z

        # Get quaternion rotation data
        block.w = data.pose.orientation.w
        block.x = data.pose.orientation.x
        block.y = data.pose.orientation.y
        block.z = data.pose.orientation.z

        # Convert quaternion to pitch, roll, yaw
        block.a = atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z) # Pitch - a
        block.b = atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z) # Roll - b
        block.c = asin(2*x*y + 2*z*w) # Yaw - c

        # Append Block object to array of objects
        blocks.append(block)

    # Publish array of Block object
    pub.publish(blocks)

    # Log number of block positions published
    #rospy.loginfo("%d block positions published.", len(block_names))

    # Wait 5 seconds before allowing next callback
    rospy.sleep(publish_rate)

def specific_block_pos(specific_model_name):
    # Use service to get position of specific block named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

    # Return ModelState object with position relative to world 
    return model_state_service(specific_model_name, "world")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass