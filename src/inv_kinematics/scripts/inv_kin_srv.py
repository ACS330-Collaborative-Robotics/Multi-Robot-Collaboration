#!/usr/bin/env python

import rospy
import tf_conversions

from trac_ik_python.trac_ik import IK

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

debug_mode = False

from time import time
from math import pi

from pathlib import Path

from inv_kinematics.srv import InvKin
from custom_msgs.msg import Joints
from geometry_msgs.msg import Pose

def trac_ik_inverse_kinematics(pose: Pose, precise_orientation, final_link_name="link6"):
    try:
        urdf_str = rospy.get_param('/robot_description')
    except KeyError:
        file = open(Path.home().as_posix() + "/catkin_ws/src/inv_kinematics/urdf/CPRMover6.urdf.xacro")
        urdf_str = file.read()

    ik_solver = IK("base_link", final_link_name, urdf_string=urdf_str)

    seed_state = [0.0]*ik_solver.number_of_joints #TODO: Update seed state to use current joint positions

    coordinate_tolerance = 1e-3 # Start with 1mm tolerance
    angle_tolerance = pi/180 # Start with 1 degree tolerance

    multiplier = 120
    if not precise_orientation:
        angle_tolerance = angle_tolerance*multiplier
    
    for angle_offset in [0, -pi, pi]: #convert to euler, +-pi rotation and convert to quaternion to test #could just be -pi? rounding?
        target_orientation_euler = list(tf_conversions.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))
        target_orientation_euler[2] = target_orientation_euler[2] + angle_offset

        quaternion_x, quaternion_y, quaternion_z, quaternion_w = tf_conversions.transformations.quaternion_from_euler(target_orientation_euler[0], target_orientation_euler[1], target_orientation_euler[2])

        for attempt_number in range(3):
            joints = ik_solver.get_ik(seed_state, pose.position.x, pose.position.y, pose.position.z, quaternion_x, quaternion_y, quaternion_z, quaternion_w, coordinate_tolerance, coordinate_tolerance, coordinate_tolerance, angle_tolerance, angle_tolerance, angle_tolerance)
            if joints != None:
                #rospy.loginfo("Inverse Kinematics - Computed sucessfully with %.0f offset on attempt %d.",180*angle_offset/pi, attempt_number+1)
                return list(joints)
    return None

def inverse_kinematics_service(req):
    #rospy.loginfo("Inverse Kinematics - Service call recieved.")

    pub = rospy.Publisher(req.state.model_name + "/joint_angles", Joints, queue_size=10)

    start_time = time()
    if req.state.reference_frame == "":
        joints = trac_ik_inverse_kinematics(req.state.pose, req.precise_orientation)
    else:
        rospy.loginfo("Inverse Kinematics - Moving %s instead of end-effector.", req.state.reference_frame)
        joints = trac_ik_inverse_kinematics(req.state.pose, req.precise_orientation, req.state.reference_frame)

    if joints is None:
        rospy.logerr("Inverse Kinematics - Failed to find a solution in %.4f for %s", time()-start_time, req.state.model_name)
        return False
    else:
        joints_display = " ".join([str(round(joint*180/pi, 2)) for joint in joints])
        ##rospy.loginfo("Inverse Kinematics - Trac IK: %s\tComputed in: %.4f", joints_display, time()-start_time)

        # Publish joint positions
        pub.publish(joints)

        ##rospy.loginfo("Inverse Kinematics - Joint positions published.\n")

        return True
    
def analyse_robot_workspace():
    x_range = [-0.5, 0.5]
    y_range = [-0.5, 0.5]
    z_values = [0, 0.05, 0.10, 0.15]

    number_of_points = 10
    number_of_points -= 1

    x_step = (max(x_range) - min(x_range))/number_of_points
    y_step = (max(y_range) - min(y_range))/number_of_points

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

            for z in z_values:
                pose_object.position.x = x
                pose_object.position.y = y
                pose_object.position.z = z

                status = trac_ik_inverse_kinematics(pose_object, True)
                if status:
                    ax.scatter(x, y, z, c='k')

        rospy.logdebug("%.0f%% Done", (x_multiplier+1)/(number_of_points+1)*100)

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
    #rospy.loginfo("Inverse Kinematics Reachability - %.3f\t%.3f\t%.3f", req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z)

    joints = trac_ik_inverse_kinematics(req.state.pose, True)

    if joints is None:
        #rospy.logwarn("Inverse Kinematics Reachability - Failed")
        return False
    else:
        #rospy.logwarn("Inverse Kinematics Reachability - True")
        return True

def main():
    if debug_mode:
        rospy.init_node('inverse_kinematics_server', log_level=rospy.DEBUG)
    else:
        rospy.init_node('inverse_kinematics_server')

    #analyse_robot_workspace()

    s1 = rospy.Service('inverse_kinematics', InvKin, inverse_kinematics_service)
    s2 = rospy.Service('inverse_kinematics_reachability', InvKin, inverse_kinematics_reachability_service)

    # Setup publish topics to avoid missing messages
    robot_namespaces = ["mover6_a", "mover6_b"]
    for robot_name in robot_namespaces:
      pub = rospy.Publisher(robot_name + "/joint_angles", Joints, queue_size=10)  

    rospy.spin()

if __name__ == "__main__":
    main()