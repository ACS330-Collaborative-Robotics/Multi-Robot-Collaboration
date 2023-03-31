import rospy
import subprocess

# Wait for the service to become available
rospy.wait_for_service('/inverse_kinematics')

# Check if the service is available
service_name = '/inverse_kinematics'
try:
    subprocess.check_output(['rosservice', 'find', service_name])
    print('Service is available')
except subprocess.CalledProcessError:
    print('Service is not available')

