import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

def callback(scan):
    # Process the laser scan data to detect humans within 30cm
    # If a human is detected, publish a message to the "emergency_stop" topic
    human_detected = False
    for r in enumerate(scan.ranges):
        #print(r)
        if r[1] < 2:
            human_detected = True
            break
        
        if rospy.is_shutdown():
            break
        
    if human_detected:
        rospy.loginfo("Human detected within 2.2m!")
        e_stop_pub.publish(True)
    else:
        e_stop_pub.publish(False)

if __name__ == '__main__':
    rospy.init_node('human_zone')
    e_stop_pub = rospy.Publisher('human_detection', Bool, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()
