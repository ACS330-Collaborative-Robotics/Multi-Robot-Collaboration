#!/usr/bin/env python
import rospy
import subprocess
import threading

import tkinter as tk
from tkinter import ttk
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (
                                    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure


from PIL import Image, ImageTk
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
import cv2

from custom_msgs.msg import Joints
from std_msgs.msg import String, Float64, Float64MultiArray


class GUI:
    def __init__(self, master):
        # simulation
        self.sim_label = tk.Label(master, text="Simulation: ")
        self.sim_label.grid(row=0, column=0, sticky="w")
        self.sim_canvas = tk.Canvas(master, width=540, height=380)
        self.sim_canvas.grid(row=1, column=0, sticky="nsew")
        self.time_label = tk.Label(master, text="")
        self.time_label.grid(row=2, column=0, sticky="w")

        # physical camera feed
        self.cam_label = tk.Label(master, text="Physical camera feed: ")
        self.cam_label.grid(row=0, column=2, sticky="w")
        self.cam_canvas = tk.Canvas(master, width=540, height=380)
        self.cam_canvas.grid(row=1, column=2, sticky="nsew")

        # potential field plot
        self.plot_label = tk.Label(master, text="Potential Field Plot: ")
        self.plot_label.grid(row=0, column=3, sticky="w")
        self.plot_canvas = tk.Canvas(master, width=540, height=380)
        self.plot_canvas.grid(row=1, column=3, sticky="nsew")

        # buttons
        self.emergency_stop_button = tk.Button(master, text="STOP", bg="red", fg="black", font=("Calibri", 10, "bold"), command=self.emergency_stop_clicked)
        self.emergency_stop_button.grid(row=3, column=0, )
        self.pause_button = tk.Button(master, text="PAUSE", bg="red", fg="black", font=("Calibri", 10, "bold"), command=self.pause_physical_clicked)
        self.pause_button.grid(row=4, column=0, )
        self.sim_preview_button = tk.Button(master, text="SIM PREVIEW", bg="yellow", fg="black", font=("Calibri", 10, "bold"), command=self.sim_preview_clicked)
        self.sim_preview_button.grid(row=5, column=0, )
    
        # status indicator lights
        # hardware connected light
        self.hardware_light = tk.Label(master,bg="red", width=2, height=1)
        self.hardware_light.grid(row=3, column=1, sticky="w")
        self.hardware_label = tk.Label(master, text="Hardware connected")
        self.hardware_label.grid(row=3, column=2, sticky="w")

        # nodes configured light
        self.nodes_light = tk.Label(master,bg="red", width=2, height=1)
        self.nodes_light.grid(row=4, column=1, sticky="w")
        self.nodes_label = tk.Label(master, text="Nodes configured")
        self.nodes_label.grid(row=4, column=2, sticky="w")

        # self.required_services = ['/inverse_kinematics', '/inverse_kinematics_reachability', '/inverse_kinematics_server/get_loggers', '/inverse_kinematics_server/set_logger_level']
        # Wait for the service to become available
        #rospy.wait_for_service('/inverse_kinematics')
        # Check if the service is available
        #service_name = '/inverse_kinematics'
        #try:
        #    subprocess.check_output(['rosservice', 'find', service_name])
        #    self.nodes_light.config(bg="green")
        #except subprocess.CalledProcessError:
        #    self.nodes_light.config(bg="red")

   

        # error status light
        self.error_light = tk.Label(master,bg="red", width=2, height=1)
        self.error_light.grid(row=5, column=1, sticky="w")
        self.error_label = tk.Label(master, text="Error status")
        self.error_label.grid(row=5, column=2, sticky="w")

        # blank space
        self.blank_label = tk.Label(master, text="")
        self.blank_label.grid(row=6, column=0, sticky="w")

        # joint angles
        self.angles= tk.Label(master, text="Joint angles from base to end-effector: ")
        self.angles.grid(row=7, column=0, sticky="w")
        self.angles_A = tk.Label(master, text="Robot A joint angles (rad): ")
        self.angles_A.grid(row=8, column=0, sticky="w")
        self.angles_B = tk.Label(master, text="Robot B joint angles (rad): ")
        self.angles_B.grid(row=9, column=0, sticky="w")


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

        # Create a listener for the physical camera 
        rospy.Subscriber('/usb_cam/image_raw', ImageMsg, self.camera_callback)

        # Create a listener for the Potential Field plot
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('')
    
    
    def camera_callback(self, msg):
        # Convert the ROS message to an OpenCV image
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Resize the image to fit in the Tkinter window
        img = cv2.resize(img, (640, 480))

        # Convert the OpenCV image to a PIL Image
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)

        # Convert the PIL Image to a Tkinter PhotoImage and display it in the canvas
        img_tk = ImageTk.PhotoImage(img)
        self.cam_canvas.create_image(0, 2, anchor=tk.NW, image=img_tk)

        self.cam_canvas.image = img_tk

   
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
        self.sim_canvas.create_image(0, 0, anchor=tk.NW, image=tk_image)
        # Keep a reference to the image to prevent it from being garbage collected
        self.sim_canvas.image = tk_image
    
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
   
    def emergency_stop_clicked(self):
        self.emergency_stop_button.config(text="START", bg="green", fg="black")
    
    def pause_physical_clicked(self):
        self.pause_button.config(text="START", bg="green", fg="black")
   
    def sim_preview_clicked(self):
        self.sim_preview_button.config(text="STOP PREVIEW", bg="red", fg="black")

   

if __name__ == '__main__':
    root = tk.Tk()
    root.geometry("{}x{}+0+0".format(root.winfo_screenwidth(), root.winfo_screenheight()))
    gui = GUI(root)
    root.mainloop()