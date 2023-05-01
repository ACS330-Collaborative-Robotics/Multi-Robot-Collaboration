#!/usr/bin/env python
import rospy
rospy.init_node('listener_gui_publisher', anonymous=True) # initialize the ROS node
import subprocess
import threading 

import tkinter as tk
from tkinter import ttk

from PIL import Image, ImageTk
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
import cv2

from custom_msgs.msg import Joints
from custom_msgs.srv import PlayPause
from std_msgs.msg import String, Float64, Float64MultiArray, Bool
from rosgraph_msgs.msg import Log
        

class GUI:
    def __init__(self, master):
        self.master = master
        self.master.title("My GUI")

        # simulation
        self.sim_label = tk.Label(master, text="Simulation: ")
        self.sim_label.grid(row=0, column=0, sticky="w")
        self.sim_canvas = tk.Canvas(master, width=900, height=675)
        self.sim_canvas.grid(row=1, column=0, sticky="nsew")
        self.time_label = tk.Label(master, text="")
        self.time_label.grid(row=2, column=0, sticky="w")

        # physical camera feed
        self.cam_label = tk.Label(master, text="Physical camera feed: ")
        self.cam_label.grid(row=0, column=2, sticky="w")
        self.cam_canvas = tk.Canvas(master, width=900, height=675)
        self.cam_canvas.grid(row=1, column=2, sticky="nsew")

        # blank space
        self.blank_label = tk.Label(master, text="")
        self.blank_label.grid(row=3, column=0, sticky="w")

        # buttons
        # emergency stop
        self.emergency_stop_button = tk.Button(master, text="Safety Stop", bg="red", fg="black", font=("Sans-serif", 10, "bold"), command=self.emergency_stop_clicked, width=10, height=1)
        self.emergency_stop_button.grid(row=4, column=0)
        self.emergency_stop_info = tk.Label(master, text="Safety stop physical and simulated robots.")
        self.emergency_stop_info.grid(row=4, column=0, sticky="e")
        self.emergency_stop_info.grid_remove()  # hide the label initially
        def show_emergencyinfo(event):
            self.emergency_stop_info.grid()  # show the label when the mouse enters the button
        def hide_emergencyinfo(event):
            self.emergency_stop_info.grid_remove()  # hide the label when the mouse leaves the button
        self.emergency_stop_button.bind("<Enter>", show_emergencyinfo)
        self.emergency_stop_button.bind("<Leave>", hide_emergencyinfo)
        
        # sim preview
        self.sim_preview_button = tk.Button(master, text="Sim Preview", bg="yellow", fg="black", font=("Sans-serif", 10, "bold"), command=self.sim_preview_clicked, width=10, height=1)
        self.sim_preview_button.grid(row=6, column=0)
        self.sim_preview_info = tk.Label(master, text="Pause physical robot and continue simulation.")
        self.sim_preview_info.grid(row=6, column=0, sticky="e")
        self.sim_preview_info.grid_remove()  # hide the label initially
        def show_previewinfo(event):
            self.sim_preview_info.grid()  # show the label when the mouse enters the button
        def hide_previewinfo(event):
            self.sim_preview_info.grid_remove()  # hide the label when the mouse leaves the button
        self.sim_preview_button.bind("<Enter>", show_previewinfo)
        self.sim_preview_button.bind("<Leave>", hide_previewinfo)

        # potential fields visualiser button 
        self.pot_visual_button = tk.Button(master, text="Visualise Algorithm", bg="yellow", fg="black", font=("Sans-serif", 10, "bold"), command=self.pot_clicked, width=15, height=1)
        self.pot_visual_button.grid(row=7, column=0)
        self.pot_visual_info = tk.Label(master, text="Viualise potential fields algorithm.")
        self.pot_visual_info.grid(row=7, column=0, sticky="e")
        self.pot_visual_info.grid_remove()  # hide the label initially
        def show_potinfo(event):
            self.pot_visual_info.grid()  # show the label when the mouse enters the button
        def hide_potinfo(event):
            self.pot_visual_info.grid_remove()  # hide the label when the mouse leaves the button
        self.pot_visual_button.bind("<Enter>", show_potinfo)
        self.pot_visual_button.bind("<Leave>", hide_potinfo)
    
        # status indicator lights
        # raspberry Pis connected light
        self.Pi_light = tk.Label(master,bg="red", width=2, height=1)
        self.Pi_light.grid(row=4, column=1)
        self.Pi_label = tk.Label(master, text="Both Raspberry Pis connected")
        self.Pi_label.grid(row=4, column=2, sticky="w", padx=(165,0))
        piServices = ['/mover6_a_p/JointJog', '/mover6_b_p/JointJog']
        try:
            output = subprocess.check_output(['rosservice', 'find'] + piServices)
            return_code = 0
            # check if service name is found in output
            if all(service in output.decode('utf-8') for service in piServices):
                self.Pi_light.config(bg="green")
            else:
                self.Pi_light.config(bg="red")
        except subprocess.CalledProcessError as e:
            output = e.output
            return_code = e.returncode
            self.Pi_light.config(bg="red")

        print(f"Output: {output}, return code: {return_code}")

        # nodes configured light
        # the aim of these lights is to firstly check that the inverse_kinematics service is running (includes controllers)
        # to check that roscore is running and that the gui can communicate with it
        self.nodes_light = tk.Label(master, bg="red", width=2, height=1)
        self.nodes_light.grid(row=5, column=1)
        self.nodes_label = tk.Label(master, text="Core nodes configured")
        self.nodes_label.grid(row=5, column=2, sticky="w", padx=(165,0))
        ik_service = '/inverse_kinematics'
        try:
            output = subprocess.check_output(['rosservice', 'list'])
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
        self.blank_label.grid(row=7, column=0)
        frame = ttk.Frame(master, relief="sunken", padding=10)
        frame.grid(row=8, column=0, columnspan=2, rowspan=3, sticky="nesw")
        
        # sim listener
        self.bridge = CvBridge() 
        rospy.Subscriber('/camera1/image_raw', ImageMsg, self.callback_video)

        # ROS spin loop
        self.thread = threading.Thread(target=rospy.spin) 
        self.thread.start()

    # error status light
    # checks the status of the most recent error
    # level 1=debug, 2=info, 3=warn, 4=error, 5=fatal
    # green if it is 1, 2 or 3
    # red if it is 4 or 5
           
    # create error status light
        self.error_light = tk.Label(self.master, bg="yellow", width=2, height=1)
        self.error_light.grid(row=6, column=1)
        self.error_label = tk.Label(self.master, text="Error status")
        self.error_label.grid(row=6, column=2, sticky="w", padx=(165,0))
        self.error_msg = tk.Text(self.master, height=5, width=50)
        self.error_msg.grid(row=7, column=1, columnspan=2, sticky="w", padx=(165,0))
        # subscribe to rosout 
        rospy.Subscriber('/rosout', Log, self.callback_error, callback_args=(self.error_msg, self.error_light))
        # Initialize the ROS publisher for the gui
        self.gui_pub = rospy.Publisher('/gui', Bool, queue_size=10)
    
    # error display
    def error_display(master):
        master.title("Error Display")
        error_light = tk.Label(master, bg="yellow", width=2, height=1)
        error_light.grid(row=0, column=1)
        error_label = tk.Label(master, text="Error status")
        error_label.grid(row=0, column=1)
        error_msg = tk.Text(master, height=5, width=50)    # error message box
        error_msg.grid(row=1, column=1, columnspan=2) 
        
    # update error log
    def callback_error(self, data, args):
        error_msg, error_light = args
        # get the most recent error message and severity level
        self.error_msgs = data.msg.split("\n")
        most_recent_error = self.error_msgs[0]
        most_recent_severity = int(data.level)
        # Only display errors of severity level 4 or 5
        if most_recent_severity < 4:
            return
        # update the error message box
        self.master.after(0, lambda: error_msg.delete(1.0, tk.END))
        self.master.after(0, lambda: error_msg.insert(tk.END, most_recent_error))
        # update the error status light
        if most_recent_severity > 3:
            error_light.config(bg="red")
        else:
            error_light.config(bg="green")
    
    # update physical camera data
    def camera_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8') # convert ROS message to OpenCV image
        img = cv2.resize(img, (900, 675)) # resize image to fit window
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # convert the OpenCV image to a PIL Image
        img = Image.fromarray(img)
        img_tk = ImageTk.PhotoImage(img) # convert the PIL Image to a Tkinter PhotoImage and display it in the canvas
        self.cam_canvas.create_image(0, 2, anchor=tk.NW, image=img_tk)
        self.cam_canvas.image = img_tk
   
     # update simulation display data
    def callback_video(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8') # ROS to cv2
        cv_image = cv2.resize(cv_image, (900, 675)) 
        pil_image = Image.fromarray(cv_image) # cv2 to PIL
        tk_image = ImageTk.PhotoImage(image=pil_image) # PIL to Tkinter-compatible
        self.sim_canvas.create_image(0, 0, anchor=tk.NW, image=tk_image)
        self.sim_canvas.image = tk_image

    # emergency stop button clicked
    def emergency_stop_clicked(self):
        if self.emergency_stop_button['text'] == 'Safety Stop': # determine state of button
            self.emergency_stop_button.config(text='Start', bg='green', fg='black')
            self.gui_pub.publish(True) # publish the message to the /gui topic
            self.emergency_stop_info.config(text="Start physical and simulated robots.")
        else:
            self.emergency_stop_button.config(text='Safety Stop', bg='red', fg='black')
            self.gui_pub.publish(False) # publish the message to the /gui topic
            self.emergency_stop_info.config(text="Safety stop physical and simulated robots.")

    # SIM PREVIEW button clicked
    def sim_preview_clicked(self):
        # this button will use the play pause service
        # when it is clicked, the physical system will pause but the simulation will continue
        # after being clicked, the 'pause' string will be passed to the play_pause_demo_service
        # then the button will change to say "STOP PREVIEW" 
        # when that is clicked, the 'play' string will be passed to the service and the button will return to say "SIM PREVIEW"
       
        # service proxy to communicate with the play pause service
        play_pause_proxy = rospy.ServiceProxy('play_pause_demo_service', PlayPause) 
        if self.sim_preview_button['text'] == 'Sim Preview': # if simulation is currently running and is not paused
            desired_state = 'pause' # then pause the simulation 
            self.sim_preview_button.config(text='Stop Preview', bg='red', fg='black') # change the button text for stopping the preview
            self.sim_preview_info.config(text="Resume physical system.")
        else:
            desired_state = 'play' # if the physical system is paused 
            self.sim_preview_button.config(text='Sim Preview', bg='yellow', fg='black')
        # call service with desired state
        try:
            response = play_pause_proxy(desired_state)
            if response.success:
                rospy.loginfo('Play/Pause service successfully executed')
            else:
                rospy.logerr('Play/Pause service failed to execute')
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    # pot fields visualiser button clicked
    def pot_clicked(self):
        if self.pot_visual_button['text'] == 'Visualise Algorithm': # determine state of button
            self.pot_visual_button.config(text='Close graph', bg='red', fg='black')
            self.gui_pub.publish(True) # publish the message to the /gui topic
            self.pot_visual_info.config(text="Close visualiser.")
            # code for graph here 
        else:
            self.pot_visual_button.config(text='Safety Stop', bg='red', fg='black')
            self.gui_pub.publish(False) # publish the message to the /gui topic
            self.pot_visual_info.config(text="Viualise potential fields algorithm.")

            
if __name__ == '__main__':
    root = tk.Tk()
    root.geometry("{}x{}+0+0".format(1920, 1080))
    gui = GUI(root)
    root.mainloop()