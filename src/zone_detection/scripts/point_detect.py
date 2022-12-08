#!/usr/bin/env python

# Name: Robot Arm Zone Detection
# Author: Andrei Codin (aacodin1@sheffield.ac.uk)

import rospy
import tf
from shapely.geometry import Point, Polygon

if __name__ == '__main__':
    rospy.init_node('point_detect')
    mover6_a_data = tf.TransformListener()
    mover6_b_data = tf.TransformListener()
    (transl_a,rot_a) = Point(mover6_a_data.lookupTransform('/kink6', '/base_link', rospy.Time(0)))
    (transl_b,rot_b) = Point(mover6_b_data.lookupTransform('/link6', '/base_link', rospy.Time(0)))
    mover6_a = (transl_a[1], transl_a[2])
    mover6_b = (transl_b[1], transl_b[2])
    coords_a = [(-0.5, -0.5), (0.5, 0), (0, 0.5), (0.5, 0.5)]
    coords_b = [(0, 0.5), (-0.5, 0.5), (0.5, -0.5), (0.5, 0)]
    zone_a = Polygon(coords_a)
    zone_b = Polygon(coords_b)
    mover6_a.within(zone_a)
    mover6_a.within(zone_b)
    mover6_b.within(zone_b)
    mover6_b.within(zone_a)

