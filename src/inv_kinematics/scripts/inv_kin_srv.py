#!/usr/bin/env python

import rospy

import ikpy.chain
import numpy as np

from math import cos, sin
from pathlib import Path

from inv_kinematics.srv import InvKin
from std_msgs.msg import Float64

def service(req):
    # x y z Co-ordinates
    x = req.state.pose.position.x
    y = req.state.pose.position.y
    z = req.state.pose.position.z
    target_position = [x, y, z]

    a = req.state.pose.orientation.x
    b = req.state.pose.orientation.y
    c = req.state.pose.orientation.z

    # Rotation - Euler angles to 3x3 rotation matrix
    a_tform = np.array([[cos(a), -sin(a), 0], [sin(a), cos(a), 0], [0, 0, 1]])
    b_tform = np.array([[cos(b), 0, sin(b)], [0, 1, 0], [-sin(b), 0, cos(b)]])
    c_tform = np.array([[1, 0, 0], [0, cos(c), -sin(c)], [0, sin(c), cos(c)]])
    ab_tform = np.dot(a_tform, b_tform)

    target_orientation = np.dot(ab_tform, c_tform)

    # Inverse Kinematics
    joints = chain.inverse_kinematics(target_position, target_orientation, orientation_mode="all")

    # Publish joint positions
    for joint_num in range(len(joints)):
        pub = rospy.Publisher(req.state.model_name + "/joint" + str(joint_num) + "_position_controller/command", Float64, queue_size=10)
        pub.publish(joints[joint_num])

    return True

def main():
    rospy.init_node('inverse_kinematics_server')

    s = rospy.Service('inverse_kinematics', InvKin, service)

    # Setup inverse kinematics object as global variable so service can use it
    global chain
    chain = ikpy.chain.Chain.from_urdf_file(Path.home().as_posix() + "/catkin_ws/src/mover6_description/urdf/CPRMover6.urdf.xacro", active_links_mask=[False, True, True, True, True, True, True])
    
    rospy.spin()

if __name__ == "__main__":
    main()