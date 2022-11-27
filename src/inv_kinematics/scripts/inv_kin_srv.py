#!/usr/bin/env python

import rospy

import ikpy.chain
import numpy as np

from math import cos, sin

import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

def service(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return (req.a + req.b)

def main():
    rospy.init_node('inverse_kinematics_server')

    #s = rospy.Service('inverse_kinematics', AddTwoInts, handle_add_two_ints)

    chain = ikpy.chain.Chain.from_urdf_file("/home/conor/catkin_ws/src/mover6_description/urdf/CPRMover6.urdf.xacro", active_links_mask=[False, True, True, True, True, True, True])

    x = 0
    y = 0.1
    z = 0.5

    a = 0
    b = 0
    c = 0

    a_tform = [[cos(a), -sin(a), 0, 0], [sin(a), cos(a), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

    b_tform = [[cos(b), 0, sin(b), 0], [0, 1, 0, 0], [-sin(b), 0, cos(b), 0], [0, 0, 0, 1]]

    c_tform = [[1, 0, 0, 0], [0, cos(c), -sin(c), 0], [0, sin(c), cos(c), 0], [0, 0, 0, 1]]

    target_position = [x, y, z]
    target_orientation = np.eye(3)

    joints = chain.inverse_kinematics(target_position, target_orientation, orientation_mode="all")
    rospy.loginfo(joints)

    cartesian_position = chain.forward_kinematics(joints)
    rospy.loginfo(cartesian_position)

    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

    chain.plot(joints, ax)
    
    matplotlib.pyplot.show()

    #rospy.spin()

if __name__ == "__main__":
    main()