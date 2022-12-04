#!/usr/bin/env python

# Name: Robot Arm Zones Coordinates Publisher
# Author: Andrei Codin (aacodin1@sheffield.ac.uk)

import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.srv import GetModelState

def talker():
    pub = rospy.Publisher('chatter2', Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    # Corners coordinates
    corner_a1 = [-0.5, -0.5]
    corner_a2 = [0.5, 0]
    corner_a3 = [0, 0.5]
    corner_a4 = [0.5, 0.5]
    corner_b1 = [0, 0.5]
    corner_b2 = [-0.5, 0.5]
    corner_b3 = [0.5, -0.5]
    corner_b4 = [0.5, 0]
    Zone = Float64MultiArray()
    zone_names = []
    for zone_name in data.name:
        if "zone" in zone_name:
            #rospy.loginfo(model_name)
            zone_names.append(zone_name)
    zones = []
    
   
    for zone_num in range(len(zone_names)):
        zone = Zone() # Empty Zone object to fill

        # Get data for specific
        data = specific_zone_pos(zone_names[zone_num]) # GetModelState
        #rospy.loginfo(data)

        zone.zone_number = int(zone_names[zone_num].replace("zone", "")) # Zone number
        zones.append(zone)

        #Setting up zones
        zone.mover6_a_zone = [corner_a1, corner_a2, corner_a3, corner_a4]
        zone.mover6_b_zone = [corner_b1, corner_b2, corner_b3, corner_b4]

    pub.publish(zones)

def specific_zone_pos(specific_model_name):
    # Use service to get position of specific zone named
    rospy.wait_for_service('gazebo/get_model_state')
    model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

    # Return ModelState object with position relative to world 
    return model_state_service(specific_model_name, "world")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
