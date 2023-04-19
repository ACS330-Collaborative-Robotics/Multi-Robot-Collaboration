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
from rosgraph_msgs.msg import Log
        

class GUI:
    def __init__(self, master):
        self.master = master
        self.master.title("My GUI")

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
        def show_emergencyinfo(event):
            self.emergency_stop_info.grid()  # show the label when the mouse enters the button
        def hide_emergencyinfo(event):
            self.emergency_stop_info.grid_remove()  # hide the label when the mouse leaves the button
        self.emergency_stop_button.bind("<Enter>", show_emergencyinfo)
        self.emergency_stop_button.bind("<Leave>", hide_emergencyinfo)
        
        # pause 
        self.pause_button = tk.Button(master, text="PAUSE", bg="red", fg="black", font=("Calibri", 10, "bold"), command=self.pause_clicked)
        self.pause_button.grid(row=5, column=0, sticky="w")
        self.pause_info = tk.Label(master, text="Pause physical and simulated robots.")
        self.pause_info.grid(row=5, column=0, sticky="e")
        self.pause_info.grid_remove()  # hide the label initially
        def show_pauseinfo(event):
            self.pause_info.grid()  # show the label when the mouse enters the button
        def hide_pauseinfo(event):
            self.pause_info.grid_remove()  # hide the label when the mouse leaves the button
        self.pause_button.bind("<Enter>", show_pauseinfo)
        self.pause_button.bind("<Leave>", hide_pauseinfo)
        
        # sim preview
        self.sim_preview_button = tk.Button(master, text="SIM PREVIEW", bg="yellow", fg="black", font=("Calibri", 10, "bold"), command=self.sim_preview_clicked)
        self.sim_preview_button.grid(row=6, column=0, sticky="w")
        self.sim_preview_info = tk.Label(master, text="Pause physical robot and continue simulation.")
        self.sim_preview_info.grid(row=6, column=0, sticky="e")
        self.sim_preview_info.grid_remove()  # hide the label initially
        def show_previewinfo(event):
            self.sim_preview_info.grid()  # show the label when the mouse enters the button
        def hide_previewinfo(event):
            self.sim_preview_info.grid_remove()  # hide the label when the mouse leaves the button
        self.sim_preview_button.bind("<Enter>", show_previewinfo)
        self.sim_preview_button.bind("<Leave>", hide_previewinfo)
    
        # status indicator lights

        # raspberry Pis connected light
        # 1 pi connected: 
        # mover6_a_p/InputChannels, 
        # /mover6_a_p/JointJog, 
        # /mover6_a_p/OutputChannels, 
        # /mover6_a_p/OutputChannels
        # /mover6_a_p/joint_states
        # /mover6_a_p/physical/joint_angles
        # /mover6_a_p/physical/moving_state
        # /mover6_a/robot_state
        
        # 2 pis connected;
        # the above topics plus:
        # /mover6_b_p_InputChannels
        # /mover6_b_p/JointJog, 
        # /mover6_b_p/OutputChannels, 
        # /mover6_b_p/OutputChannels
        # /mover6_b_p/joint_states
        # /mover6_b_p/physical/joint_angles
        # /mover6_b_p/physical/moving_state
        # /mover6_b/robot_state

        self.Pi_light = tk.Label(master,bg="red", width=2, height=1)
        self.Pi_light.grid(row=4, column=1, sticky="w")
        self.Pi_label = tk.Label(master, text="Both Raspberry Pis connected")
        self.Pi_label.grid(row=4, column=2, sticky="w")

        piServices = ' '.join(['/mover6_a_p/InputChannels',
        '/mover6_a_p/JointJog',
        '/mover6_a_p/OutputChannels',
        '/mover6_a_p/joint_states',
        '/mover6_a_p/physical/joint_angles',
        '/mover6_a_p/physical/moving_state',
        '/mover6_a/robot_state',
        '/mover6_b_p/InputChannels',
        '/mover6_b_p/JointJog',
        '/mover6_b_p/OutputChannels',
        '/mover6_b_p/joint_states',
        '/mover6_b_p/physical/joint_angles',
        '/mover6_b_p/physical/moving_state',
        '/mover6_b/robot_state'])

        try:
            output = subprocess.check_output(['rosservice', 'find', piServices])
            return_code = 0
            # check if service name is found in output
            if piServices in output.decode('utf-8'):
                self.Pi_light.config(bg="green")
            else:
                self.Pi_light.config(bg="red")
        except subprocess.CalledProcessError as e:
            output = e.output
            return_code = e.returncode
            self.nodes_light.config(bg="red")
        print(f"Output: {output}, return code: {return_code}")

       # nodes configured light
        # the aim of these lights is to firstly check that the inverse_kinematics service is running (includes controllers)
        # to check that roscore is running and that the gui can communicate with it
        self.nodes_light = tk.Label(master, bg="red", width=2, height=1)
        self.nodes_light.grid(row=5, column=1, sticky="w")
        self.nodes_label = tk.Label(master, text="Core nodes configured")
        self.nodes_label.grid(row=5, column=2, sticky="w")
        
        # self.required_services = ['/inverse_kinematics', '/inverse_kinematics_reachability', '/inverse_kinematics_server/get_loggers', '/inverse_kinematics_server/set_logger_level']
        # Wait for the service to become available
        ik_service = '/inverse_kinematics'
        try:
            output = subprocess.check_output(['rosservice', 'find', ik_service])
            return_code = 0
            # check if service name is found in output
            if ik_service in output.decode('utf-8'):
                self.nodes_light.config(bg="green")
            else:
                self.nodes_light.config(bg="red")
        except subprocess.CalledProcessError as e:
            output = e.output
            return_code = e.returncode
            self.nodes_light.config(bg="red")
        print(f"Output: {output}, return code: {return_code}")

        
        # blank space
        self.blank_label = tk.Label(master, text="")
        self.blank_label.grid(row=7, column=0, sticky="w")
        frame = ttk.Frame(master, relief="sunken", padding=10)
        frame.grid(row=8, column=0, columnspan=2, rowspan=3, sticky="nesw")

        
        # Create a listener for the clock topic
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/clock', Float64, self.callback_time)

        # Create a listener for the video topic
        self.bridge = CvBridge()
        rospy.Subscriber('/camera1/image_raw', ImageMsg, self.callback_video)

        # Start a separate thread for the ROS spin loop
        self.thread = threading.Thread(target=rospy.spin)
        self.thread.start()

 
    # error status light
    # checks the status of the most recent error
    # level 1=debug, 2=info, 3=warn, 4=error, 5=fatal
    # yellow if it is 1, 2 or 3
    # orange if it is 4
    # red if it is 5
    def callback(data, error_msg, error_light):
        # get the most recent error message and severity level
        error_msgs = data.msg.split("\n")
        most_recent_error = error_msgs[-2]
        most_recent_severity = int(data.level)

        # update the error message box
        error_msg.delete(1.0, tk.END)
        error_msg.insert(tk.END, most_recent_error)

        # update the error status light
        if most_recent_severity <= 3:
            error_light.config(bg="yellow")
        elif most_recent_severity == 4:
            error_light.config(bg="orange")
        else:
            error_light.config(bg="red")

    def error_display(master):
        master.title("Error Display")

        error_light = tk.Label(master, bg="yellow", width=2, height=1)
        error_light.grid(row=0, column=0, sticky="w")
        error_label = tk.Label(master, text="Error status")
        error_label.grid(row=0, column=1, sticky="w")

        # error message box
        error_msg = tk.Text(master, height=5, width=50)
        error_msg.grid(row=1, column=0, columnspan=2)

        # subscribe to the rosout topic
        sub = rospy.Subscriber("/rosout", Log, lambda data: callback(data, error_msg, error_light))    
    
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
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8') # ROS to cv2
        cv_image = cv2.resize(cv_image, (640, 480)) 
        pil_image = Image.fromarray(cv_image) # cv2 to PIL
        tk_image = ImageTk.PhotoImage(image=pil_image) # PIL to Tkinter-compatible
        self.sim_canvas.create_image(0, 0, anchor=tk.NW, image=tk_image)
        self.sim_canvas.image = tk_image
    
    # update simulation time 
    def callback_time(self, data):
        time_secs = data.clock.secs + data.clock.nsecs / 1e9
        time_str = "{:.1f}".format(time_secs)
        self.time_label.configure(text="Simulation run-time (s): " + time_str)

        # Schedule the function call to update the simulation time label again after 100 milliseconds
        self.time_label.after(100, lambda: self.callback_time(data))

    
    def emergency_stop_clicked(self):
        subprocess.call(['/usr/bin/python3', '/home/wiks2/catkin_ws/src/e_stop/scripts/e_stop.py'])
        self.change_button_state()

    def change_button_state(self):
        self.emergency_stop_button.config(text="START", bg="green", fg="black")

    def pause_clicked(self):
        self.pause_button.config(text="START", bg="green", fg="black")
    
    def sim_preview_clicked(self):
        self.sim_preview_button.config(text="STOP PREVIEW", bg="red", fg="black")

   
if __name__ == '__main__':
    root = tk.Tk()
    root.geometry("{}x{}+0+0".format(root.winfo_screenwidth(), root.winfo_screenheight()))
    gui = GUI(root)
    root.mainloop()