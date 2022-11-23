#!/usr/bin/env python
# license removed for brevity
import rospy
from inv_kinematics.msg import ArmPos
from gazebo_msgs.srv import GetModelState
from random import randint

def talker():
    robot_namespaces = ["mover6_a", "mover6_b"]
    pub = []
    for robot in robot_namespaces:
        pub.append(rospy.Publisher("/command_pos", ArmPos, queue_size=2))

    rospy.init_node('joint_movement_demo')
    T_period = 2
    rate = rospy.Rate(1/T_period)
    
    a = 0
    b = 3.14
    c = 0

    while not rospy.is_shutdown():
        for robot_num in range(len(robot_namespaces)):
            block = "block" + str(randint(1,20))

            xyz_pos = specific_block_pos(block, robot_namespaces[robot_num])

            arm_pos = ArmPos()
            arm_pos.robot_namespace = robot_namespaces[robot_num]

            arm_pos.x = xyz_pos[0]
            arm_pos.y = xyz_pos[1]
            arm_pos.z = xyz_pos[2] + 0.05

            arm_pos.a = a
            arm_pos.b = b
            arm_pos.c = c

            pub[robot_num].publish(arm_pos)
            rospy.loginfo(robot_namespaces[robot_num] + " " + block)

            rate.sleep()

def specific_block_pos(specific_model_name, reference_model_name):
    # Use service to get position of specific block named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    data = model_state_service(specific_model_name, reference_model_name).pose.position

    # Return ModelState object with position relative to world 
    return [data.x, data.y, data.z]

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass