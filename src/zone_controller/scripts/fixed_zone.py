#!/usr/bin/env python

# Name: Robot Arm Zones Coordinates Publisher
# Author: Andrei Codin (aacodin1@sheffield.ac.uk)

import rospy
from zone_controller.msg import Coord, Zone, Zones

def talker():
    pub = rospy.Publisher('zones', Zones, queue_size=10)
    rospy.init_node('zone_publisher', anonymous=True)

    # mover6_a co-ordinates
    corner_a1 = Coord()
    corner_a1.coordinate = [0.5, 0]
    corner_a2 = Coord()
    corner_a2.coordinate = [-0.5, 0]
    corner_a3 = Coord()
    corner_a3.coordinate = [-0.5, 0.5]
    corner_a4 = Coord()
    corner_a4.coordinate = [0.5, 0.5]

    # mover6_b co-ordinates
    corner_b1 = Coord()
    corner_b1.coordinate = [0.5, 0]
    corner_b2 = Coord()
    corner_b2.coordinate = [0.5, -0.5]
    corner_b3 = Coord()
    corner_b3.coordinate = [-0.5, -0.5]
    corner_b4 = Coord()
    corner_b4.coordinate = [-0.5, 0]

    zone_data = [[corner_a1, corner_a2, corner_a3, corner_a4], [corner_b1, corner_b2, corner_b3, corner_b4]]

    robot_namespaces = ["mover6_a", "mover6_b"]
    
    r = rospy.Rate(1)
        
    while not(rospy.is_shutdown()):
        zones = []
        for robot_num in range(len(robot_namespaces)):
            zone = Zone()
            zone.zone_name = robot_namespaces[robot_num] + "_zone"

            zone.zone_coords = zone_data[robot_num]

            zones.append(zone)

        pub.publish(zones)

        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
