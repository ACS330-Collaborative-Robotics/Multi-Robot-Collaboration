#!/usr/bin/env python

import rospy
import tf_conversions

import ikpy.chain
from trac_ik_python.trac_ik import IK

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

disable_fk = False
try:
    import kinpy as kp
except ModuleNotFoundError:
    print("Inverse Kinematics - kinpy not found, forward kinematics diagnostics disabled.")
    disable_fk = True

from time import time
from math import pi

from pathlib import Path

from inv_kinematics.srv import InvKin
from custom_msgs.msg import Joints
from geometry_msgs.msg import Pose

def forward_kinematics(joint_values):
    chain = kp.build_chain_from_urdf(open(Path.home().as_posix() + "/catkin_ws/src/inv_kinematics/urdf/CPRMover6.urdf.xacro").read())

    link_names = chain.get_joint_parameter_names()
    
    joints = {}
    for joint_num in range(6):
        joints[link_names[joint_num]] = joint_values[joint_num]

    cartesian_coords = chain.forward_kinematics(joints)

    return cartesian_coords["link6"]

def ikpy_inverse_kinematics(pose: Pose):
    chain = ikpy.chain.Chain.from_urdf_file(Path.home().as_posix() + "/catkin_ws/src/mover6_description/urdf/CPRMover6.urdf.xacro", active_links_mask=[False, True, True, True, True, True, True])

    # x y z Co-ordinates
    target_position = [pose.position.x, pose.position.y, pose.position.z]

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
    try:
        urdf_str = rospy.get_param('/robot_description')
    except KeyError:
        file = open(Path.home().as_posix() + "/catkin_ws/src/inv_kinematics/urdf/CPRMover6.urdf.xacro")
        urdf_str = file.read()

    ik_solver = IK("base_link", "link6", urdf_string=urdf_str)

    seed_state = [0.0]*ik_solver.number_of_joints #TODO: Update seed state to use current joint positions

    coordinate_tolerance = 1e-3 # Start with 1mm tolerance
    angle_tolerance = pi/180 # Start with 1 degree tolerance

    joints = ik_solver.get_ik(seed_state, pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, coordinate_tolerance, coordinate_tolerance, coordinate_tolerance, angle_tolerance, angle_tolerance, angle_tolerance)

    #multiplier = 10
    #while joints is None:
    #    coordinate_tolerance = coordinate_tolerance * multiplier
    #
    #    print("Inverse Kinematics - Trac Ik: Failed to find solution, increasing tolerance by 10 times to", coordinate_tolerance, angle_tolerance)
    #
    #    joints = ik_solver.get_ik(seed_state, pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, coordinate_tolerance, coordinate_tolerance, coordinate_tolerance, angle_tolerance, angle_tolerance, angle_tolerance)
        
    if joints is None:
        return None
    else:
        return list(joints)

def inverse_kinematics_service(req):
    print("Inverse Kinematics - Service call recieved.")
    pub = rospy.Publisher(req.state.model_name + "/joint_angles", Joints, queue_size=10)

    start_time = time()
    joints = trac_ik_inverse_kinematics(req.state.pose)

    if joints is None:
        print("Inverse Kinematics - Failed to find a solution in ", round(time()-start_time, 4))
        print("")
        return False
    else:
        print("Inverse Kinematics - Trac IK: ", joints, " Computed in: ", round(time()-start_time, 4))

        if disable_fk != True:
            # Understanding IK accuracy 
            target_position = [req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z]
            target_orientation = [req.state.pose.orientation.w, req.state.pose.orientation.x, req.state.pose.orientation.y, req.state.pose.orientation.z]
            target = target_position + target_orientation

            end_effector_position = forward_kinematics(joints)
            final_position = list(end_effector_position.pos)
            final_orientation = list(end_effector_position.rot)
            final = final_position + final_orientation

            print("")

            print("Type\tx\ty\tz\trx\try\trz\trw")

            print("Target\t", end="")
            for value in target: {print(round(value,3), "\t", end="")}
            print("")

            print("Final\t", end="")
            for value in final: {print(round(value,3), "\t", end="")}
            print("")

            print("Diff\t", end="")
            for value_pos in range(len(final)):
                print(round(target[value_pos] - final[value_pos] ,3), "\t", end="")
            print("")        
        
        # Publish joint positions
        pub.publish(joints)

        print("Inverse Kinematics - Joint positions published.\n")

        return True
    
def analyse_robot_workspace():
    x_range = [-0.5, 0.5]
    y_range = [-0.5, 0.5]
    z_range = [-0.1, 0.3]

    number_of_points = 5
    number_of_points -= 1

    x_step = (max(x_range) - min(x_range))/number_of_points
    y_step = (max(y_range) - min(y_range))/number_of_points
    z_step = (max(z_range) - min(z_range))/number_of_points

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    pose_object = Pose()

    orientation_in_euler = [0,180*pi/180,0]
    orientation = tf_conversions.transformations.quaternion_from_euler(orientation_in_euler[0], orientation_in_euler[1], orientation_in_euler[2])
    
    pose_object.orientation.x = orientation[0]
    pose_object.orientation.y = orientation[1]
    pose_object.orientation.z = orientation[2]
    pose_object.orientation.w = orientation[3]

    for x_multiplier in range(0, number_of_points+1):
        x = x_multiplier*x_step + min(x_range)

        for y_multiplier in range(0, number_of_points+1):
            y = y_multiplier*y_step + min(y_range)

            for z_multiplier in range(0, number_of_points+1):
                z = z_multiplier*z_step + min(z_range)
                
                pose_object.position.x = x
                pose_object.position.y = y
                pose_object.position.z = z

                status = trac_ik_inverse_kinematics(pose_object)
                if status:
                    ax.scatter(x, y, z, c='k')

        print((x_multiplier+1)/(number_of_points+1)*100, "% Done")

        if rospy.is_shutdown():
            break

    ax.scatter(0, 0, 0, c="r")

    ax.axis(x_range + y_range)

    ax.set_title('Plot of Mover6 Reachable Workspace\nwith End Effector downwards facing')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()

def inverse_kinematics_reachability_service(req):
    joints = trac_ik_inverse_kinematics(req.state.pose)

    if joints is None:
        return False
    else:
        return True

def main():
    rospy.init_node('inverse_kinematics_server')

    analyse_robot_workspace()

    s1 = rospy.Service('inverse_kinematics', InvKin, inverse_kinematics_service)
    s2 = rospy.Service('inverse_kinematics_reachability', InvKin, inverse_kinematics_reachability_service)

    # Setup publish topics to avoid missing messages
    robot_namespaces = ["mover6_a", "mover6_b"]
    for robot_name in robot_namespaces:
      pub = rospy.Publisher(robot_name + "/joint_angles", Joints, queue_size=10)  

    rospy.spin()

if __name__ == "__main__":
    main()