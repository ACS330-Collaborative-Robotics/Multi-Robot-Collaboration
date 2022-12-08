#!/usr/bin/env python

# Name: Robot Arm Zone Detection
# Author: Andrei Codin (aacodin1@sheffield.ac.uk)

import rospy
from shapely.geometry import Point, Polygon

def talker():
    mover6_a = Point()
    mover6_b = Point()
    coords_a = [(-0.5, -0.5), (0.5, 0), (0, 0.5), (0.5, 0.5)]
    coords_b = [(0, 0.5), (-0.5, 0.5), (0.5, -0.5), (0.5, 0)]
    zone_a = Polygon(coords_a)
    zone_b = Polygon(coords_b)
