#!/usr/bin/env python

# Name: Emergency Stop Topic
# Author: Tom Richards (tmrichards1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Bool
from sys import argv

print("Emergency stop script started.")

def main():
    rospy.init_node('e_stop', anonymous=True)

    #Define the publishers
    E_stop_pub = rospy.Publisher("/emergency_stop_manual", Bool, queue_size=10)

    # TODO: Make this dynamic based on number of robots
    mover6_a_pub = rospy.Publisher("mover6_a/pause_physical", Bool, queue_size=10)
    mover6_b_pub = rospy.Publisher("mover6_b/pause_physical", Bool, queue_size=10)

    # Allowing all to start
    E_stop = False

    mover6_a = False
    mover6_b = False

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        # Input from terminal
        input_value = input("1) E-Stop \n2) mover6a \n3) mover6b \n->")
        # TODO: Add error handling for malicious inputs

        #Publishing 
        if input_value == "1":
            E_stop = not E_stop
            E_stop_pub.publish(E_stop)

        elif input_value == "2":
            mover6_a = not mover6_a
            mover6_a_pub.publish(mover6_a)

        elif input_value == "3":
            mover6_b = not mover6_b
            mover6_b_pub.publish(mover6_b)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass