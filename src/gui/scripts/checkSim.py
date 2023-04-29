#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
import cv2

def callback_video(data):
    cv_image = CvBridge().imgmsg_to_cv2(data, desired_encoding='bgr8') # ROS to cv2
    cv_image = cv2.resize(cv_image, (900, 675))
    cv2.imshow("Simulation Video", cv_image)
    cv2.waitKey(1)

def test_video_loading():
    rospy.init_node('test_video_loading', anonymous=True)
    rospy.Subscriber('/camera1/image_raw', ImageMsg, callback_video)
    rospy.spin()

if __name__ == '__main__':
    test_video_loading()