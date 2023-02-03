#!/usr/bin/env python

# Name: Move all robots joint's through full range of motion
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Float64
from basic_movement.msg import Joints

def talker():
    robot_namespaces = ["mover6_a", "mover6_b"]
    pub = []
    for robot in robot_namespaces:
        pub.append(rospy.Publisher(robot + "/joint_angles", Joints, queue_size=10))

    #limits = [[-130, 130], [-50, 60], [-110, 75], [-140, 140], [-70, 60], [-120, 120]]
    limits = [[0,0], [0,0], [-100, 65], [0,0], [0,0], [0,0]]
    # Radians limits 3dp = [[-2.269,2.269],[-0.873,1.047],[-1.920,1.309],[-2.443,2.443],[-1.221,1.047],[-2.094,2.094]]
    points = 9

    rospy.init_node('joint_movement_demo')
    T_period = 10
    rate = rospy.Rate(1/T_period)

    currentPoint = 0

    while not rospy.is_shutdown():
        for robot in pub:
            pos = []
            for joint in range(6):
                pos.append(currentPoint*(limits[joint][1] - limits[joint][0])/(points-1) + limits[joint][0])#*3.14/180 #converts to radians
            robot.publish(pos)
        currentPoint = (currentPoint + 1) % points
        rospy.loginfo(pos)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass