#!/usr/bin/env python

# Name: Emergency Stop Topic
# Author: Tom Richards (tmrichards1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Bool
from sys import argv

def main():
    rospy.init_node('e_stop')

    #Defineing the publishers
    phicGripperEStoppub = rospy.Publisher("/emergancy_stop", Bool, queue_size=10) # TODO: Needs Testing
    simGrippub = rospy.Publisher("sim_gripper_stop", Bool, queue_size=10) # TODO: Needs Implometing
    simRobopub = rospy.Publisher("sim_robot_stop", Bool, queue_size=10) # TODO: Needs Implometing
    mover6apub = rospy.Publisher("mover6a/e_stop", Bool, queue_size=10) # TODO: Needs Testing
    mover6bpub = rospy.Publisher("mover6b/_stop", Bool, queue_size=10) # TODO: Needs Testing

    # allwoing all to start
    phicGripperEStop = False #TODO
    phicGripper = False #TODO
    simGripper = False #TODO
    simRobot = False #TODO
    mover6a = False #TODO
    mover6b = False #TODO


    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        # input from terminoal
        spegetti = input("1) E-Stop \n2) Gisical grippers \n3) Simulation Gripper \n4) Sumulation Gripper \n5) mover6a \n6) mover6b \n->")
        #Publishing out
        if spegetti == "1":
            phicGripperEStop = not phicGripperEStop
            phicGripperEStoppub.publish(phicGripperEStop)
        elif spegetti == "2":
            phicGripper = not phicGripper
            PhiscGrippub.publish(phicGripper)
        elif spegetti == "3":
            simGripper = not simGripper
            simGrippub.publish(simGripper)
        elif spegetti == "4":
            simRobot = not simRobot
            simRobopub.publish(simRobot)
        elif spegetti == "5":
            mover6a = not mover6a
            mover6apub.publish(mover6a)
        elif spegetti == "6":
            mover6b = not mover6b
            mover6bpub.publish(mover6b)

        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass