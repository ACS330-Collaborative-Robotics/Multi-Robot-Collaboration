#!/usr/bin/env python
# license removed for brevity
import rospy
from gazebo_msgs import ModelState
import random

def talker():
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    rospy.init_node('place_blocks')

    rospy.loginfo("")

    pub.publish(pos)

        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


#rostopic info /gazebo/set_model_state
#Type: gazebo_msgs/ModelState

#Publishers: None

#Subscribers: 
# * /gazebo (http://conor-PC:44767/)


#rosmsg info gazebo_msgs/ModelState
#string model_name
#geometry_msgs/Pose pose
#  geometry_msgs/Point position
#    float64 x
#    float64 y
#    float64 z
#  geometry_msgs/Quaternion orientation
#    float64 x
#    float64 y
#    float64 z
#    float64 w
#geometry_msgs/Twist twist
#  geometry_msgs/Vector3 linear
#    float64 x
#    float64 y
#    float64 z
#  geometry_msgs/Vector3 angular
#    float64 x
#    float64 y
#    float64 z
#string reference_frame
