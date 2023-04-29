#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Float64

class VideoDisplay:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera1/image_raw', ImageMsg, self.callback_video)
        self.time_sub = rospy.Subscriber('/clock', Float64, self.callback_time)
        self.last_time = None

    def callback_video(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8') # ROS to cv2
        cv_image = cv2.resize(cv_image, (450, 335))
        cv2.imshow("Simulation Video", cv_image)
        cv2.waitKey(1)
        if self.last_time is not None:
            time_elapsed = rospy.get_time() - self.last_time
            print("Time elapsed: {:.1f} seconds".format(time_elapsed))
        self.last_time = rospy.get_time()

    def callback_time(self, data):
        time_secs = data.data
        time_str = "{:.1f}".format(time_secs)
        print("Simulation run-time (s):", time_str)

if __name__ == '__main__':
    rospy.init_node('video_display', anonymous=True)
    video_display = VideoDisplay()
    rospy.spin()