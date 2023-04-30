#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Float64
import tkinter as tk

class VideoDisplay:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera1/image_raw', ImageMsg, self.callback_video)
        self.time_sub = rospy.Subscriber('/clock', Float64, self.callback_time)
        self.last_time = None
        self.time_label = None  # initialize time_label as an attribute

        # clock listener
        rospy.Subscriber('/clock', Float64, self.callback_time)

    def create_widgets(self):
        # create the time_label widget
        self.time_label = tk.Label(text="Simulation run-time (s): ")
        self.time_label.pack()

    def callback_video(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8') # ROS to cv2
        cv_image = cv2.resize(cv_image, (450, 335))
        cv2.imshow("Simulation Video", cv_image)
        cv2.waitKey(1)
        if self.last_time is not None:
            time_elapsed = rospy.get_time() - self.last_time
            print("Time elapsed: {:.1f} seconds".format(time_elapsed))
        self.last_time = rospy.get_time()


    # update simulation time 
    def callback_time(self, data):
        time_secs = data.clock.secs + data.clock.nsecs / 1e9
        time_str = "{:.1f}".format(time_secs)
        print("Simulation run-time (s): " + time_str)
        if self.time_label is None:
            self.create_widgets()  # create the time_label widget if it hasn't been created yet
        self.time_label.config(text="Simulation run-time (s): " + time_str) # update the text of the time_label widget
     
if __name__ == '__main__':
    rospy.init_node('video_display', anonymous=True)
    video_display = VideoDisplay()
    rospy.spin()