#!/usr/bin/env python

# Name: Emergency Stop Topic
# Author: Tom Richards (tmrichards1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Bool
from sys import argv

global phicGripper
global simGripper
global phicRobot
global simRobot
phicGripper = False
simGripper = False
phicRobot = False
simRobot = False
