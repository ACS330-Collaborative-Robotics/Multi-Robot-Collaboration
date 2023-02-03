#!/usr/bin/env python

# Name: Robot Arm Zone Detection
# Author: Andrei Codin (aacodin1@sheffield.ac.uk)

import rospy
#import tf
from shapely.geometry import Point, Polygon
from zone_controller.msg import Zones

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data.zone_data)

    print(data.zone_data[0])

    # To be replaced with Forward Kinematics
    mover6_a = Point(0.2, 0.1)
    mover6_b = Point(-0.2, -0.1)

    # Get Zone data
    coords_a = data.zone_data[0]
    coords_b = data.zone_data[1] # TODO: Make this work for N zones

    # Zone detection inness
    zone_a = Polygon(coords_a.zone_coords) # TODO: Convert from object to 2D Array
    zone_b = Polygon(coords_b.zone_coords)

    pos_a1 = mover6_a.within(zone_a)
    pos_a2 = mover6_a.within(zone_b)
    pos_b1 = mover6_b.within(zone_b)
    pos_b2 = mover6_b.within(zone_a)

def listener():
    while not rospy.is_shutdown():
        rospy.init_node('point_detect', anonymous=True)
        rospy.Subscriber("zones", Zones, callback)

        #mover6_a_data = tf.TransformListener()
        #mover6_b_data = tf.TransformListener()
        #(transl_a,rot_a) = mover6_a_data.lookupTransform('/link6', '/base_link', rospy.Time(0))
        #(transl_b,rot_b) = mover6_b_data.lookupTransform('/link6', '/base_link', rospy.Time(0))

        rospy.spin()


if __name__ == '__main__':
    listener()