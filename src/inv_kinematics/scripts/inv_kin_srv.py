#!/usr/bin/env python

import rospy

import ikpy.chain
import numpy as np

from math import cos, sin

import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

from gazebo_msgs.msg import ModelState

import time

def service(req):
    x = req.pose.position.x
    y = req.pose.position.y
    z = req.pose.position.z
    target_position = [x, y, z]

    a = req.pose.orientation.x
    b = req.pose.orientation.y
    c = req.pose.orientation.z

    a_tform = np.array([[cos(a), -sin(a), 0], [sin(a), cos(a), 0], [0, 0, 1]])
    b_tform = np.array([[cos(b), 0, sin(b)], [0, 1, 0], [-sin(b), 0, cos(b)]])
    c_tform = np.array([[1, 0, 0], [0, cos(c), -sin(c)], [0, sin(c), cos(c)]])
    ab_tform = np.dot(a_tform, b_tform)

    target_orientation = np.dot(ab_tform, c_tform)
    print(target_orientation)

    joints = chain.inverse_kinematics(target_position, target_orientation, orientation_mode="all")
    print(joints)

    cartesian_position = chain.forward_kinematics(joints)
    print(cartesian_position)

    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

    chain.plot(joints, ax)

    matplotlib.pyplot.xlim([-0.5, 0.5])
    matplotlib.pyplot.ylim([-0.5, 0.5])
    
    matplotlib.pyplot.show()

    return joints

def main():
    rospy.init_node('inverse_kinematics_server')

    #s = rospy.Service('inverse_kinematics', ModelState, service)

    global chain
    chain = ikpy.chain.Chain.from_urdf_file("/home/conor/catkin_ws/src/mover6_description/urdf/CPRMover6.urdf.xacro", active_links_mask=[False, True, True, True, True, True, True])
    
    state = ModelState()

    state.pose.position.x = 0.1
    state.pose.position.y = 0.1
    state.pose.position.z = 0.5

    state.pose.orientation.x = 0
    state.pose.orientation.y = 0
    state.pose.orientation.z = 0

    service(state)

    #rospy.spin()

if __name__ == "__main__":
    main()