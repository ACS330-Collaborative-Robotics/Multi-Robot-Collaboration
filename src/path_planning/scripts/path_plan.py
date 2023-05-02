#! /usr/bin/env python

# Name: Path Planner Runner
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy

from path_planner import PathPlanner

import actionlib
from path_planning.msg import PathPlanAction, PathPlanResult


class PathPlannerServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('path_planner', PathPlanAction, self.path_plan, False)

        self.server.start()

        rospy.spin()

    def path_plan(self, goal):
        # Initialise PathPlanner object
        rospy.logwarn("Path Planner - %s move %s to target: %.2f %.2f %.2f", goal.robot_name, goal.block_name,  goal.end_pos.position.x,  goal.end_pos.position.y,  goal.end_pos.position.z)

        pathPlanner = PathPlanner.PathPlanner(goal.robot_name, goal.block_name, goal.end_pos)

        temp = PathPlanResult()
        path_planner_status = pathPlanner.pathPlan()
        temp.success = path_planner_status

        rospy.loginfo("Path Planner - %s - Completed - %s\n", goal.robot_name, goal.block_name)

        if path_planner_status:
            self.server.set_succeeded(temp)
        else:
            self.server.set_aborted(temp)

if __name__ == "__main__":
    debug_mode = False
    if debug_mode:
        rospy.init_node('path_planner_server', log_level=rospy.DEBUG)
    else:
        rospy.init_node('path_planner_server')

    server = PathPlannerServer()
    rospy.spin()
    