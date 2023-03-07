#! /usr/bin/env python

# Name: Path Planner Runner
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy

from geometry_msgs.msg import Pose
from path_planning.srv import PathPlan

from path_planner import PathPlanner

def path_plan(req):
    # Initialise PathPlanner object
    print("Path planner recieved instruction:\t", req.robot_name, "\t",req.block_name, "\t", req.end_pos.position.x, "\t", req.end_pos.position.y, "\t", req.end_pos.position.z)
    pathPlanner = PathPlanner.PathPlanner(req.robot_name, req.block_name, req.end_pos)

    # Run Path planner and return state
    return pathPlanner.pathPlan()

def main():
    rospy.init_node('path_planner_server')

    s = rospy.Service('path_planner', PathPlan, path_plan)

    rospy.spin()

if __name__ == "__main__":
    main()