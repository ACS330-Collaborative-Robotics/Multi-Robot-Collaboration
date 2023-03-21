#!/usr/bin/python3

# Name: Block Position Publisher from apriltag_ros transforms
# Author: Joseph Fields (jfields1@sheffield.ac.uk)
import rospy
import sys
import math
import tf2_ros
import geometry_msgs.msg
from block_controller.msg import Block, Blocks
import tf.transformations


if __name__ == '__main__':
    rospy.init_node('tf2_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer) #create transform listener
    pub = rospy.Publisher('/blocks_pos_cam', Blocks, queue_size=10)
    rate = rospy.Rate(float(sys.argv[1])) #default polling and publishing rate

    rospy.loginfo("Node started: tf2_listener - polling rate %s Hz",sys.argv[1])
    while not rospy.is_shutdown():
        blocks = []
        for i in range(2,35): #cycle through tags 2-34
            block = Block()
            tagID='tag_'+str(i)
            try:
                trans = tfBuffer.lookup_transform('mover6aBase', tagID, rospy.Time(0)) # get transform between tag_0 abd
                block.block_number=i
                block.x=trans.transform.translation.x #unit: meters
                block.y=trans.transform.translation.y
                block.z=trans.transform.translation.z

                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([float(trans.transform.rotation.x),float(trans.transform.rotation.y),float(trans.transform.rotation.z),float(trans.transform.rotation.w)])
                block.a=pitch
                block.b=roll
                block.c=yaw
                
                blocks.append(block)
            except (tf2_ros.ConnectivityException,tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue  
        pub.publish(blocks)
        
        rate.sleep()

