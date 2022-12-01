#! /usr/bin/env python

# Name: Path Planner Runner
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

from geometry_msgs.msg import Pose
import rospy

from path_planner import *

def main():
    robot_ns = "mover6_a"
    block_name = "block0"
    end_pos = Pose()

    pathPlanner = PathPlanner.PathPlanner(robot_ns, block_name, end_pos)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
