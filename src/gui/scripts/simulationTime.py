#!/usr/bin/env python

import rospy
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

from std_msgs.msg import String, Float64, Float64MultiArray
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
import threading
import cv2

class GUI:
    def __init__(self, master):
         # Create a label widget and add it to the window
        self.label1 = tk.Label(master, text="Simulation Run-time (s): ")
        self.label1.grid(row=0, column=0, sticky="w")
        # Create a label widget to display the simulation time
        self.time_label = tk.Label(master, text="0.0")
        self.time_label.grid(row=0, column=1, sticky="w")

        # Create a canvas to display the video
        self.canvas = tk.Canvas(master, width=640, height=480)
        self.canvas.grid(row=1, column=0, columnspan=2)

        # Create a listener for the clock topic
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/clock', Float64, self.callback_time)

        # Create a listener for the video topic
        self.bridge = CvBridge()
        rospy.Subscriber('/camera1/image_raw', ImageMsg, self.callback_video)

        # Start a separate thread for the ROS spin loop
        self.thread = threading.Thread(target=rospy.spin)
        self.thread.start()

    # update simulation time 
    def callback_time(self, data):
        time_secs = data.clock.secs + data.clock.nsecs / 1e9
        self.time_label.configure(text=str(time_secs))

    # update video frame
    def callback_video(self, data):
        # Convert the ROS image message to a cv2 image
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        # Resize the image to fit in the canvas
        cv_image = cv2.resize(cv_image, (640, 480))
        # Convert the cv2 image to a PIL image
        pil_image = Image.fromarray(cv_image)
        # Convert the PIL image to a Tkinter-compatible image
        tk_image = ImageTk.PhotoImage(image=pil_image)
        # Display the image on the canvas
        self.canvas.create_image(0, 0, anchor=tk.NW, image=tk_image)
        # Keep a reference to the image to prevent it from being garbage collected
        self.canvas.image = tk_image
        
if __name__ == '__main__':
    root = tk.Tk()
    gui = GUI(root)
    root.mainloop()