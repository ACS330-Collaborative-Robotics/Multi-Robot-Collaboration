#!/usr/bin/env python

import rospy
import tkinter as tk
from tkinter import ttk

from std_msgs.msg import String, Float64, Float64MultiArray
import threading

class GUI:
    def __init__(self, master):
        
        # Create a label widget and add it to the window
        self.label1 = tk.Label(master, text="Simulation Run-time (s): ")
        self.label1.grid(row=0, column=0, sticky="w")
        # Create a label widget to display the simulation time
        self.time_label = tk.Label(master, text="0.0")
        self.time_label.grid(row=0, column=1, sticky="w")

        # Mover6 A
        self.label2 = tk.Label(master, text="Mover6 A joint angles (rad) : ")
        self.label2.grid(row=9, column=0, sticky="w")
        self.joint1_label_a = tk.Label(master, text="Joint 1: ")
        self.joint1_label_a.grid(row=10, column=0, sticky="w")
        self.joint2_label_a = tk.Label(master, text="Joint 2:")
        self.joint2_label_a.grid(row=11, column=0, sticky="w")
        self.joint3_label_a = tk.Label(master, text="Joint 3: ")
        self.joint3_label_a.grid(row=12, column=0, sticky="w")
        self.joint4_label_a = tk.Label(master, text="Joint 4:")
        self.joint4_label_a.grid(row=13, column=0, sticky="w")
        self.joint5_label_a = tk.Label(master, text="Joint 5:")
        self.joint5_label_a.grid(row=14, column=0, sticky="w")
        self.joint6_label_a = tk.Label(master, text="Joint 6:")
        self.joint6_label_a.grid(row=15, column=0, sticky="w")

        # Mover 6 B
        self.label2 = tk.Label(master, text="Mover6 B joint angles (rad) : ")
        self.label2.grid(row=16, column=0, sticky="w")
        self.joint1_label_b = tk.Label(master, text="Joint 1: ")
        self.joint1_label_b.grid(row=17, column=0, sticky="w")
        self.joint2_label_b = tk.Label(master, text="Joint 2:")
        self.joint2_label_b.grid(row=18, column=0, sticky="w")
        self.joint3_label_b = tk.Label(master, text="Joint 3: ")
        self.joint3_label_b.grid(row=19, column=0, sticky="w")
        self.joint4_label_b = tk.Label(master, text="Joint 4:")
        self.joint4_label_b.grid(row=20, column=0, sticky="w")
        self.joint5_label_b = tk.Label(master, text="Joint 5:")
        self.joint5_label_b.grid(row=21, column=0, sticky="w")
        self.joint6_label_b = tk.Label(master, text="Joint 6:")
        self.joint6_label_b.grid(row=22, column=0, sticky="w")
        

       

        # ROS initialization
        rospy.init_node('listener', anonymous=True)

        # Subscribe to the clock topic
        rospy.Subscriber('/clock', Float64, self.callback)

        # Subscribe to the joint angles topics for Mover6 A and B
        rospy.Subscriber('/mover6_a/joint_angles', Float64MultiArray, self.update_mover6_a)
        rospy.Subscriber('/mover6_b/joint_angles', Float64MultiArray, self.update_mover6_b)

        
        # Start a separate thread for the ROS spin loop
        self.thread = threading.Thread(target=rospy.spin)
        self.thread.start()


    def callback(self, data):
        time_secs = data.clock.secs + data.clock.nsecs / 1e9

        # Update the simulation time label with the new data
        self.time_label.configure(text=str(time_secs))


    def update_mover6_a(self, data):
        joint_angles = data.data
        rospy.loginfo(joint_angles)
        # Update the joint angle labels for Mover6 A
        self.joint1_label_a.configure(text=str(joint_angles[0]))
        self.joint2_label_a.configure(text=str(joint_angles[1]))
        self.joint3_label_a.configure(text=str(joint_angles[2]))
        self.joint4_label_a.configure(text=str(joint_angles[3]))
        self.joint5_label_a.configure(text=str(joint_angles[4]))
        self.joint6_label_a.configure(text=str(joint_angles[5]))

    def update_mover6_b(self, data):
        joint_angles = data.data

        # Update the joint angle labels for Mover6 B
        self.joint1_label_b.configure(text=str(joint_angles[0]))
        self.joint2_label_b.configure(text=str(joint_angles[1]))
        self.joint3_label_b.configure(text=str(joint_angles[2]))
        self.joint4_label_b.configure(text=str(joint_angles[3]))
        self.joint5_label_b.configure(text=str(joint_angles[4]))
        self.joint6_label_b.configure(text=str(joint_angles[5]))


if __name__ == '__main__':
    root = tk.Tk()
    gui = GUI(root)
    root.mainloop()