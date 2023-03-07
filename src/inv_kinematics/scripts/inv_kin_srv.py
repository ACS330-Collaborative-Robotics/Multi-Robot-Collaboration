#!/usr/bin/env python

import rospy
import tf_conversions

import ikpy.chain

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

def service(req):
    print("Inverse Kinematics - Service call recieved.")
    pub = rospy.Publisher(req.state.model_name + "/joint_angles", Joints, queue_size=10)

    joints = ikpy_inverse_kinematics(req.state.pose)
    
    print("Inverse Kinematics - Publishing: ", joints)

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