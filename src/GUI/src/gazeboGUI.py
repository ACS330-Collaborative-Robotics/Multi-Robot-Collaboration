#!/usr/bin/env python
import rospy
import cv_bridge
import tkinter
from sensor_msgs.msg import Image
from PIL import Image as PILImage, ImageTk

class CameraViewer:
    def __init__(self):
        # Initialize the node
        rospy.init_node('camera_viewer', anonymous=True)

        # Create a subscriber to receive camera images
        self.image_sub = rospy.Subscriber('camera/image_raw', Image, self.image_callback)

        # Initialize the OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # Create a GUI window
        self.window = tkinter.Tk()

        # Create a canvas to display the image
        self.canvas = tkinter.Canvas(self.window, width=640, height=480)
        self.canvas.pack()

    def image_callback(self, msg):
        try:
            # Convert the ROS message to a numpy array
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Convert the numpy array to a PIL image
            pil_image = PILImage.fromarray(cv_image)

            # Display the PIL image in the GUI
            self.img = ImageTk.PhotoImage(pil_image)
            self.canvas.create_image(0, 0, anchor=tkinter.NW, image=self.img)

        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)

    def run(self):
        # Start the GUI event loop
        self.window.mainloop()

if __name__ == '__main__':
    viewer = CameraViewer()
    viewer.run()