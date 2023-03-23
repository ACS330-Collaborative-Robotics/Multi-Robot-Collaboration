#! /usr/bin/env python

# Name: Path Planner Runner
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy

from geometry_msgs.msg import Pose
from path_planning.srv import PathPlan

from path_planner import PathPlanner

def path_plan(req):
    # Initialise PathPlanner object
    rospy.loginfo("Path planner recieved instruction:\t%s\t%s\t\t%.2f\t%.2f\t%.2f", req.robot_name, req.block_name,  req.end_pos.position.x,  req.end_pos.position.y,  req.end_pos.position.z)
    pathPlanner = PathPlanner.PathPlanner(req.robot_name, req.block_name, req.end_pos)

    # Run Path planner and return state
    return pathPlanner.pathPlan()

def main():
    rospy.init_node('path_planner_server')

    s = rospy.Service('path_planner', PathPlan, path_plan)

    rospy.spin()

if __name__ == "__main__":
    main()