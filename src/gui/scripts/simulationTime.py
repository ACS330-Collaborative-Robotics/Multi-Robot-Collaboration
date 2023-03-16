#!/usr/bin/env python

import rospy
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from custom_msgs.msg import Joints

from std_msgs.msg import String, Float64, Float64MultiArray
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
import threading
import cv2

class GUI:
    def __init__(self, master):
        # create a label for the simulation
        self.label1 = tk.Label(master, text="Simulation: ")
        self.label1.grid(row=0, column=0, sticky="w")
        # Create a canvas to display the video
        self.canvas = tk.Canvas(master, width=640, height=480)
        self.canvas.grid(row=1, column=0, columnspan=2)
        
        # Create a label widget to display the simulation time
        self.time_label = tk.Label(master, text="")
        self.time_label.grid(row=3, column=0, sticky="w")
        
        # create a blank label
        self.label1 = tk.Label(master, text="")
        self.label1.grid(row=4, column=0, sticky="w")

        # Create labels for joint angles
        self.angles= tk.Label(master, text="Joint angles from base to end-effector: ")
        self.angles.grid(row=5, column=0, sticky="w")
        self.angles_A = tk.Label(master, text="Robot A joint angles (rad): ")
        self.angles_A.grid(row=6, column=0, sticky="w")
        self.angles_B = tk.Label(master, text="Robot B joint angles (rad): ")
        self.angles_B.grid(row=7, column=0, sticky="w")

        # Create a listener for the clock topic
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/clock', Float64, self.callback_time)

        # Create a listener for the video topic
        self.bridge = CvBridge()
        rospy.Subscriber('/camera1/image_raw', ImageMsg, self.callback_video)

        # Create a listener for the joints
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/mover6_a/joint_angles', Joints, self.callback_joint_a)
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/mover6_b/joint_angles', Joints, self.callback_joint_b)

        # Start a separate thread for the ROS spin loop
        self.thread = threading.Thread(target=rospy.spin)
        self.thread.start()

   
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
    
    # update simulation time 
    def callback_time(self, data):
        time_secs = data.clock.secs + data.clock.nsecs / 1e9
        time_str = "{:.1f}".format(time_secs)
        self.time_label.configure(text="Simulation run-time (s): " + time_str)

        # Schedule the function call to update the simulation time label again after 100 milliseconds
        self.time_label.after(100, lambda: self.callback_time(data))

    # update joints for robot A
    def callback_joint_a(self, data):
        joint_angles = data.joints
        joint_angles_str = ["{:.1f}".format(joint_angle) for joint_angle in joint_angles]
        self.angles_A.configure(text="Robot A joint angles (rad): " + ", ".join(joint_angles_str))

    # update joints for robot B
    def callback_joint_b(self, data):
        joint_angles = data.joints
        joint_angles_str = ["{:.1f}".format(joint_angle) for joint_angle in joint_angles]
        self.angles_B.configure(text="Robot B joint angles (rad): " + ", ".join(joint_angles_str))

   
        
if __name__ == '__main__':
    root = tk.Tk()
    gui = GUI(root)
    root.mainloop()