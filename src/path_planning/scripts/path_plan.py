#! /usr/bin/env python

# Name: Path Planner Runner
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy

from geometry_msgs.msg import Pose
from path_planning.srv import PathPlan

from path_planner import PathPlanner

def path_plan(req):
    robot_ns = req.robot_name
    block_name = req.block_name
    end_pos = req.end_pos

    pathPlanner = PathPlanner.PathPlanner(robot_ns, block_name, end_pos)

    return pathPlanner.pathPlan()

def main():
    rospy.init_node('path_planner_server')

    s = rospy.Service('path_planner', PathPlan, path_plan)

    rospy.spin()

if __name__ == "__main__":
    main()