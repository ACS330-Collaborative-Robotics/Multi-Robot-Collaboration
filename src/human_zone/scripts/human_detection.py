
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

def callback(scan):
    # Process the laser scan data to detect humans within 30cm
    # If a human is detected, publish a message to the "human_detection" topic
    human_detected = False
    for i, r in enumerate(scan.ranges):
        if r < 0.3:
            human_detected = True
            break
    if human_detected:
        rospy.loginfo("Human detected within 30cm!")
        lidar_pub.publish(True)
    else:
        lidar_pub.publish(False)

if __name__ == '__main__':
    rospy.init_node('human_zone')
    lidar_pub = rospy.Publisher('human_detection', Bool, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()
