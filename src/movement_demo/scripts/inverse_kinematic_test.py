#!/usr/bin/env python

# Name: Inverse Kinematics Test
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy
import tf_conversions
from math import pi

from inv_kinematics.srv import InvKin, InvKinRequest 
from geometry_msgs.msg import Pose

def main():
    # Initialise ROS node
    rospy.init_node('joint_movement_demo')

    #############################
    ## Configurable Parameters ##
    #############################

    robot_name = "mover6_a"

    cartesian_coordinates = [0.25, 0, 0.2]
    orientation_in_euler = [pi, 0, pi]

    #############################

    # Setup inverse_kinematics service
    rospy.wait_for_service('inverse_kinematics')
    inverse_kinematics_service = rospy.ServiceProxy('inverse_kinematics', InvKin)

    rospy.loginfo("Calling Inverse Kinematics Service.")

    # Initialise and fill ArmPos object
    inverse_kinematics_object = InvKinRequest()
    inverse_kinematics_object.state.model_name = robot_name
    inverse_kinematics_object.state.reference_frame = ""

    end_pos = Pose()
    
    end_pos.position.x = cartesian_coordinates[0]
    end_pos.position.y = cartesian_coordinates[1]
    end_pos.position.z = cartesian_coordinates[2]

    quat = tf_conversions.transformations.quaternion_from_euler(orientation_in_euler[0], orientation_in_euler[1], orientation_in_euler[2])

    end_pos.orientation.x = quat[0]
    end_pos.orientation.y = quat[1]
    end_pos.orientation.z = quat[2]
    end_pos.orientation.w = quat[3]

    inverse_kinematics_object.state.pose = end_pos

    status = inverse_kinematics_service(inverse_kinematics_object).success

    if status:
        rospy.loginfo("Inverse Kinematics Success.")
    else:
        rospy.logerr("Inverse Kinematics Failed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass