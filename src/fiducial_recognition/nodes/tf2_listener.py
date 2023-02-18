#!/usr/bin/python3
import rospy

import math
import tf2_ros
import geometry_msgs.msg
from block_controller.msg import block, blocks


if __name__ == '__main__':
    rospy.init_node('tf2_listener')
    print('tf2 Listener Online')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        blocks = []
        for i in range(2,35):
            block = Block()
            tagID='tag_'+str(i)
            try:
                trans = tfBuffer.lookup_transform('tag_0', tagID, rospy.Time(0))
                block.block_number=i
                block.x=trans.transform.translation.x
                block.y=trans.transform.translation.y
                block.z=trans.transform.translation.z

                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(float(trans.transform.rotation.x),float(trans.transform.rotation.y),float(trans.transform.rotation.z),float(trans.transform.rotation.w))
                block.a=pitch
                block.b=roll
                block.c=yaw
                
                blocks.append(block)
            except (tf2_ros.ConnectivityException,tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue  
         
        rate.sleep()

