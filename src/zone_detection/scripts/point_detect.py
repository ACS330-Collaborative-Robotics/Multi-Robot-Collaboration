#!/usr/bin/env python

# Name: Robot Arm Zone Detection
# Author: Andrei Codin (aacodin1@sheffield.ac.uk)

import rospy
import tf
from shapely.geometry import Point, Polygon
from zone_detection.msg import zone_check

def talker():
    while not rospy.is_shutdown():
        pub = rospy.Publisher('zones', zone_check, queue_size=10)
        rospy.init_node('point_detect')
        mover6_a_data = tf.TransformListener()
        mover6_b_data = tf.TransformListener()
        (transl_a,rot_a) = Point(mover6_a_data.lookupTransform('/link6', '/base_link', rospy.Time(0)))
        (transl_b,rot_b) = Point(mover6_b_data.lookupTransform('/link6', '/base_link', rospy.Time(0)))
        mover6_a = (transl_a[1], transl_a[2])
        mover6_b = (transl_b[1], transl_b[2])
        coords_a = [(-0.5, -0.5), (0.5, 0), (0, 0.5), (0.5, 0.5)]
        coords_b = [(0, 0.5), (-0.5, 0.5), (0.5, -0.5), (0.5, 0)]
        zone_a = Polygon(coords_a)
        zone_b = Polygon(coords_b)
        pos_a1 = mover6_a.within(zone_a)
        pos_a2 = mover6_a.within(zone_b)
        pos_b1 = mover6_b.within(zone_b)
        pos_b2 = mover6_b.within(zone_a)

        zone_data = [[pos_a1, pos_a2], [pos_b1, pos_b2]]

        robot_namespaces = ["mover6_a", "mover6_b"]

        r = rospy.Rate(1)

        while not(rospy.is_shutdown()):
            zones=[]
            for robot_num in range(len(robot_namespaces)):
                zone = zone_check()
                zone.zone_name = robot_namespaces[robot_num] + "_zone"

                zone.zone_coords = zone_data[robot_num]

                zones.append(zone)

            pub.publish(zones)

            r.sleep()

