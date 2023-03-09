#!/usr/bin/env python

import rospy
import tf_conversions

import ikpy.chain
from trac_ik_python.trac_ik import IK
from time import time
from math import pi

from pathlib import Path

from inv_kinematics.srv import InvKin
from custom_msgs.msg import Joints
from geometry_msgs.msg import Pose

def ikpy_inverse_kinematics(pose: Pose):
    chain = ikpy.chain.Chain.from_urdf_file(Path.home().as_posix() + "/catkin_ws/src/mover6_description/urdf/CPRMover6.urdf.xacro", active_links_mask=[False, True, True, True, True, True, True])

    # x y z Co-ordinates
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    target_position = [x, y, z]

    # Convert from quaternions to 3x3 transformation matrix
    target_orientation_quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    target_orientation = tf_conversions.transformations.quaternion_matrix(target_orientation_quaternion)
    
    # Cut 4x4 to 3x3
    target_orientation = target_orientation[0:3, 0:3]

    # Inverse Kinematics
    joints = chain.inverse_kinematics(target_position, target_orientation, orientation_mode="all")
    joints = list(joints[1:])

    return joints

def trac_ik_inverse_kinematics(pose: Pose):
    urdf_str = rospy.get_param('/robot_description')

    ik_solver = IK("base_link", "link6", urdf_string=urdf_str)

    seed_state = [0.0]*ik_solver.number_of_joints

    #print(pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

    coordinate_tolerance = 1e-4 # Start with 1mm tolerance
    angle_tolerance = pi/180/10 # Start with 0.1 degree tolerance

    joints = ik_solver.get_ik(seed_state, pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, coordinate_tolerance, coordinate_tolerance, coordinate_tolerance, angle_tolerance, angle_tolerance, angle_tolerance)

    multiplier = 10
    while joints is None:
        coordinate_tolerance = coordinate_tolerance * multiplier
        angle_tolerance = angle_tolerance * multiplier

        print("Inverse Kinematics - Trac Ik: Failed to find solution, increasing tolerance by 10 times to", coordinate_tolerance, angle_tolerance)

        joints = ik_solver.get_ik(seed_state, pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, coordinate_tolerance, coordinate_tolerance, coordinate_tolerance, angle_tolerance, angle_tolerance, angle_tolerance)
        

    if joints is None:
        return None
    else:
        return list(joints)

def service(req):
    print("Inverse Kinematics - Service call recieved.")
    pub = rospy.Publisher(req.state.model_name + "/joint_angles", Joints, queue_size=10)

    start_time = time()
    joints = trac_ik_inverse_kinematics(req.state.pose)
    print("Inverse Kinematics - Trac IK: ", joints, " Computed in: ", round(time()-start_time, 4))

    # IF trac_ik does not find an adequate solution, use ikpy to find a nearby approximation
    if joints is None:
        print("Inverse Kinematics - ERROR - Accurate IK not found, using fallback method.")
        start_time = time()
        joints = ikpy_inverse_kinematics(req.state.pose)
        print("Inverse Kinematics - ikpy: ", joints, " Computed in: ", round(time()-start_time, 4))

    # Publish joint positions
    pub.publish(joints)

    print("Inverse Kinematics - Joint positions published.\n")

    return True

def main():
    rospy.init_node('inverse_kinematics_server')

    s = rospy.Service('inverse_kinematics', InvKin, service)
    
    rospy.spin()

if __name__ == "__main__":
    main()