#!/usr/bin/python3

import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':

        rospy.init_node('tag0_mover6a_tf2_broadcaster')
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "tag_0"
        static_transformStamped.child_frame_id = "mover6aBase"

        static_transformStamped.transform.translation.x = float(sys.argv[1])
        static_transformStamped.transform.translation.y = float(sys.argv[2])
        static_transformStamped.transform.translation.z = float(sys.argv[3])

        quat = tf.transformations.quaternion_from_euler(
                   float(sys.argv[4]),float(sys.argv[5]),float(sys.argv[6]))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)
        rospy.loginfo("Node started: static_tf2_broadcaster - 'tag_0' to 'mover6aBase' is x:%s y:%s z:%s",static_transformStamped.transform.translation.x,static_transformStamped.transform.translation.y,static_transformStamped.transform.translation.z)
        rospy.spin()