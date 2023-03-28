#! /usr/bin/env python

# Name: Path Planner Runner
# Author: Conor Nichols (cjnichols1@sheffield.ac.uk)

import rospy

from path_planner import PathPlanner

import actionlib
from path_planning.msg import PathPlanAction


class PathPlannerServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('path_planner', PathPlanAction, self.path_plan, False)

        self.server.start()

        rospy.spin()

    def path_plan(self, goal):
        # Initialise PathPlanner object
        rospy.loginfo("Path planner recieved instruction:\t%s\t%s\t\t%.2f\t%.2f\t%.2f", goal.robot_name, goal.block_name,  goal.end_pos.position.x,  goal.end_pos.position.y,  goal.end_pos.position.z)

        pathPlanner = PathPlanner.PathPlanner(goal.robot_name, goal.block_name, goal.end_pos)

        path_planner_status = pathPlanner.pathPlan()

        if path_planner_status:
            self.server.set_succeeded()
        else:
            self.server.set_aborted()

        # Run Path planner and return state
        return 

if __name__ == "__main__":
    rospy.init_node('path_planner_server')
    server = PathPlannerServer()
    rospy.spin()