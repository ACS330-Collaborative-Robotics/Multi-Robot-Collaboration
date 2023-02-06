#!/usr/bin/env python

# Name: Robot Arm Zone Detection
# Author: Andrei Codin (aacodin1@sheffield.ac.uk)

import rospy
import tf
import time
from shapely.geometry import Point, Polygon
from zone_controller.msg import Zones

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data.zone_data)

    # To be replaced with Forward Kinematics
    mover6_a = Point(0.2, 0.1)
    mover6_b = Point(-0.2, -0.1)

    # Get Zone data
    coords_a = data.zone_data[0]
    coords_b = data.zone_data[1] # TODO: Make this work for N zones

    # Zone creation
    coords_za = [(coords_a.zone_coords[0].coordinate),(coords_a.zone_coords[1].coordinate),(coords_a.zone_coords[2].coordinate),(coords_a.zone_coords[3].coordinate)]
    coords_zb = [(coords_b.zone_coords[0].coordinate),(coords_b.zone_coords[1].coordinate),(coords_b.zone_coords[2].coordinate),(coords_b.zone_coords[3].coordinate)]

    # Zone detection inness
    zone_a = Polygon(coords_za) # TODO: Convert from object to 2D Array
    zone_b = Polygon(coords_zb)
    print(zone_a)
    pos_a1 = mover6_a.within(zone_a)
    pos_a2 = mover6_a.within(zone_b)
    pos_b1 = mover6_b.within(zone_b)
    pos_b2 = mover6_b.within(zone_a)
    print(pos_a1)
    print(pos_a2)
    print(pos_b1)
    print(pos_b2)

def listener():
    rospy.init_node('point_detect')
    rospy.Subscriber("zones", Zones, callback)

    # Use TF for forward kinematics to find joint positions
    listener = tf.TransformListener()

    while listener.canTransform('mover6_a/world', 'mover6_a/link6', rospy.Time(0)) == False:
        rospy.sleep(0.01)

    try:
        # Calculate transfrom from robot base to link 6
        (trans,rot) = listener.lookupTransform('mover6_a/world', 'mover6_a/link6', rospy.Time(0)) 

        # Tranform from world to robot base
        #(trans,rot) = listener.lookupTransform('world', 'mover6_a_base', rospy.Time(0))

        print(trans)
        print(rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Error.")


if __name__ == '__main__':
    listener()