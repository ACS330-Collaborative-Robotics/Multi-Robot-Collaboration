#!/usr/bin/env python

import rospy
import tkinter as tk
from tkinter import ttk

from std_msgs.msg import String, Float64
import threading

class GUI:
    def __init__(self, master):
        # Create a label widget and add it to the window
        self.label1 = tk.Label(master, text="Simulation Run-time (s): ")
        self.label1.grid(row=0, column=0, sticky="w")
        # Create a label widget to display the simulation time
        self.time_label = tk.Label(master, text="0.0")
        self.time_label.grid(row=0, column=1, sticky="e")
        
        # Create a label widget and add it to the window
        self.label1 = tk.Label(master, text="Simulation Run-time (s): ")
        self.label1.grid(row=0, column=0, sticky="w")

       

        # ROS initialization
        rospy.init_node('listener', anonymous=True)

        # Subscribe to the clock topic
        rospy.Subscriber('/clock', Float64, self.callback)

        # Start a separate thread for the ROS spin loop
        self.thread = threading.Thread(target=rospy.spin)
        self.thread.start()

    def callback(self, data):
        time_secs = data.clock.secs + data.clock.nsecs / 1e9

        # Update the simulation time label with the new data
        self.time_label.configure(text=str(time_secs))


if __name__ == '__main__':
    root = tk.Tk()
    gui = GUI(root)
    root.mainloop()