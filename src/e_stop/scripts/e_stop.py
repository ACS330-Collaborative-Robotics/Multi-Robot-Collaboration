# #!/usr/bin/env python

# # Name: Emergency Stop Topic
# # Author: Tom Richards (tmrichards1@sheffield.ac.uk)

# import rospy
# from std_msgs.msg import Bool
# from sys import argv

# def main():
#     rospy.init_node('e_stop')

#     #Define the publishers
#     E_stop_pub = rospy.Publisher("/emergency_stop", Bool, queue_size=10)

#     # TODO: Make this dynamic based on number of robots
#     mover6_a_pub = rospy.Publisher("mover6_a/pause_physical", Bool, queue_size=10)
#     mover6_b_pub = rospy.Publisher("mover6_b/pause_physical", Bool, queue_size=10)

#     # Allowing all to start
#     E_stop = False

#     mover6_a = False
#     mover6_b = False

#     rate = rospy.Rate(10) # 10 Hz

#     while not rospy.is_shutdown():
#         # Input from terminal
#         input_value = input("1) E-Stop \n2) mover6a \n3) mover6b \n->")
#         # TODO: Add error handling for malicious inputs

#         #Publishing 
#         if input_value == "1":
#             E_stop = not E_stop
#             E_stop_pub.publish(E_stop)

#         elif input_value == "2":
#             mover6_a = not mover6_a
#             mover6_a_pub.publish(mover6_a)

#         elif input_value == "3":
#             mover6_b = not mover6_b
#             mover6_b_pub.publish(mover6_b)

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass

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
                rospy.loginfo('Stopping arms as human is detected in workspace')
            if self.gui_pressed:
                rospy.loginfo('Stopping arms as stop button in GUI is pressed')
        else:
            self.emergency_stop_pub.publish(False)

if __name__ == '__main__':
    try:
        EStopController()
    except rospy.ROSInterruptException:
        pass
