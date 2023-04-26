#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

class EStopController:
    def __init__(self):
        rospy.init_node('e_stop_controller')

        # Initialize publishers and subscribers
        self.human_detection_sub = rospy.Subscriber('/human_detection', Bool, self.human_detection_callback)
        self.gui_sub = rospy.Subscriber('/gui', Bool, self.gui_callback)
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=10)

        # Initialize state variables
        self.human_detected = False
        self.gui_pressed = False

        # Run the node
        rospy.spin()

    def human_detection_callback(self, msg):
        # Update the human detection state
        self.human_detected = msg.data

        # Check if an emergency stop is required
        self.check_emergency_stop()

    def gui_callback(self, msg):
        # Update the GUI state
        self.gui_pressed = msg.data

        # Check if an emergency stop is required
        self.check_emergency_stop()

    def check_emergency_stop(self):
        # Check if a human has been detected or the GUI has been pressed
        if self.human_detected or self.gui_pressed:
            # Publish an emergency stop signal
            self.emergency_stop_pub.publish(True)
            
            # Print a message indicating why the emergency stop was triggered
            if self.human_detected:
                rospy.logerr('Stopping arms as human is detected within 2.2m')
            if self.gui_pressed:
                rospy.logerr('Stopping arms as stop button in GUI is pressed')
        else:
            # Publish a resume signal
            self.emergency_stop_pub.publish(False)

if __name__ == '__main__':
    try:
        EStopController()
    except rospy.ROSInterruptException:
        pass
