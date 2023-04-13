#!/usr/bin/env python
import rospy
import subprocess
import threading
import subprocess

import tkinter as tk
from tkinter import ttk

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

        # blank space
        self.blank_label = tk.Label(master, text="")
        self.blank_label.grid(row=3, column=0, sticky="w")

        # buttons
        # emergency stop
        self.emergency_stop_button = tk.Button(master, text="STOP", bg="red", fg="black", font=("Calibri", 10, "bold"), command=self.emergency_stop_clicked)
        self.emergency_stop_button.grid(row=4, column=0, sticky="w")
        self.emergency_stop_info = tk.Label(master, text="Emergency stop physical and simulated robots.")
        self.emergency_stop_info.grid(row=4, column=0, sticky="e")
        self.emergency_stop_info.grid_remove()  # hide the label initially
        def show_tooltip(event):
            self.emergency_stop_info.grid()  # show the label when the mouse enters the button
        def hide_tooltip(event):
            self.emergency_stop_info.grid_remove()  # hide the label when the mouse leaves the button
        self.emergency_stop_button.bind("<Enter>", show_tooltip)
        self.emergency_stop_button.bind("<Leave>", hide_tooltip)
        
        # pause 
        self.pause_button = tk.Button(master, text="PAUSE", bg="red", fg="black", font=("Calibri", 10, "bold"), command=self.emergency_stop_clicked)
        self.pause_button.grid(row=5, column=0, sticky="w")
        self.pause_info = tk.Label(master, text="Pause physical and simulated robots.")
        self.pause_info.grid(row=5, column=0, sticky="e")
        self.pause_info.grid_remove()  # hide the label initially
        def show_tooltip(event):
            self.pause_info.grid()  # show the label when the mouse enters the button
        def hide_tooltip(event):
            self.pause_info.grid_remove()  # hide the label when the mouse leaves the button
        self.pause_button.bind("<Enter>", show_tooltip)
        self.pause_button.bind("<Leave>", hide_tooltip)
        
        # sim preview
        self.sim_preview_button = tk.Button(master, text="SIM PREVIEW", bg="yellow", fg="black", font=("Calibri", 10, "bold"), command=self.sim_preview_clicked)
        self.sim_preview_button.grid(row=6, column=0, sticky="w")
        self.sim_preview_info = tk.Label(master, text="Pause physical robot and continue simulation.")
        self.sim_preview_info.grid(row=6, column=0, sticky="e")
        self.sim_preview_info.grid_remove()  # hide the label initially
        def show_tooltip(event):
            self.sim_preview_info.grid()  # show the label when the mouse enters the button
        def hide_tooltip(event):
            self.sim_preview_info.grid_remove()  # hide the label when the mouse leaves the button
        self.sim_preview_button.bind("<Enter>", show_tooltip)
        self.sim_preview_button.bind("<Leave>", hide_tooltip)
    
        # status indicator lights
        # hardware connected light
        # to check that the mover6s are connected to the system
        # to check that the raspberry pis are connected by checking the relevant topics are running
        self.hardware_light = tk.Label(master,bg="red", width=2, height=1)
        self.hardware_light.grid(row=4, column=1, sticky="w")
        self.hardware_label = tk.Label(master, text="Hardware connected")
        self.hardware_label.grid(row=4, column=2, sticky="w")


       # nodes configured light
        # the aim of these lights is to firstly check that the inverse_kinematics service is running (includes controllers)
        # to check that roscore is running and that the gui can communicate with it
        self.nodes_light = tk.Label(master, bg="red", width=2, height=1)
        self.nodes_light.grid(row=5, column=1, sticky="w")
        self.nodes_label = tk.Label(master, text="Nodes configured")
        self.nodes_label.grid(row=5, column=2, sticky="w")
        # self.required_services = ['/inverse_kinematics', '/inverse_kinematics_reachability', '/inverse_kinematics_server/get_loggers', '/inverse_kinematics_server/set_logger_level']
        # Wait for the service to become available
        service_name = '/inverse_kinematics'
        try:
            output = subprocess.check_output(['rosservice', 'find', service_name])
            return_code = 0
            # check if service name is found in output
            if service_name in output.decode('utf-8'):
                self.nodes_light.config(bg="green")
            else:
                self.nodes_light.config(bg="red")
        except subprocess.CalledProcessError as e:
            output = e.output
            return_code = e.returncode
            self.nodes_light.config(bg="red")
        print(f"Output: {output}, return code: {return_code}")

        # error status light
        # checks the status of the most recent error
        # level 1=debug, 2=info, 3=warn, 4=error, 5=fatal
        # yellow if it is 1, 2 or 3
        # orange if it is 4
        # red if it is 5
        self.error_light = tk.Label(master,bg="red", width=2, height=1)
        self.error_light.grid(row=6, column=1, sticky="w")
        self.error_label = tk.Label(master, text="Error status")
        self.error_label.grid(row=6, column=2, sticky="w")

        # blank space
        self.blank_label = tk.Label(master, text="")
        self.blank_label.grid(row=7, column=0, sticky="w")

        # joint angles
        self.angles= tk.Label(master, text="Joint angles from base to end-effector: ")
        self.angles.grid(row=8, column=0, sticky="w")
        self.angles_A = tk.Label(master, text="Robot A joint angles (rad): ")
        self.angles_A.grid(row=9, column=0, sticky="w")
        self.angles_B = tk.Label(master, text="Robot B joint angles (rad): ")
        self.angles_B.grid(row=10, column=0, sticky="w")
        

        # Create a listener for the clock topic
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/clock', Float64, self.callback_time)

        # Create a listener for the video topic
        self.bridge = CvBridge()
        rospy.Subscriber('/camera1/image_raw', ImageMsg, self.callback_video)

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
            subprocess.call(['/usr/bin/python3', '/home/wiks2/catkin_ws/src/e_stop/scripts/e_stop.py'])
            self.change_button_state()

    def change_button_state(self):
        self.emergency_stop_button.config(text="START", bg="green", fg="black")

    def pause_button_state(self):
        self.pause_button.config(text="START", bg="green", fg="black")
    
    def sim_preview_clicked(self):
        self.sim_preview_button.config(text="STOP PREVIEW", bg="red", fg="black")

   
if __name__ == '__main__':
    root = tk.Tk()
    root.geometry("{}x{}+0+0".format(root.winfo_screenwidth(), root.winfo_screenheight()))
    gui = GUI(root)
    root.mainloop()