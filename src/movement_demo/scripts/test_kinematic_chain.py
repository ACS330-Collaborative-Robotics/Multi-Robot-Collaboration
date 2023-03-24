import rospy

from path_planning.srv import PathPlan
from geometry_msgs.msg import Pose

def choose_block():
    # Setup path_planner service
    rospy.wait_for_service('path_planner')
    path_service = rospy.ServiceProxy('path_planner', PathPlan)

    # Declare ROS Node name
    rospy.init_node('test_kinematic_chain')

    robot_name = "mover6_a"
    
    block_name = "block18"

    end_pos = Pose()
    
    end_pos.position.x = 0.2
    end_pos.position.y = 0
    end_pos.position.z = 0.2
    
    end_pos.orientation.x = 0
    end_pos.orientation.y = 0.707
    end_pos.orientation.z = 0
    end_pos.orientation.w = 0.707
    
    rospy.loginfo("Test Kinematic Chain - Calling path planner")
    
    try:
        success = path_service(block_name, end_pos, robot_name)

        if not(success):
            rospy.loginfo("Block Selection - Service call returned False.")
            
    except rospy.ServiceException as e:
        rospy.loginfo("Block Selection - Service call failed: %s"%e)

if __name__ == '__main__':
    try:
        choose_block()
    except rospy.ROSInterruptException:
        pass
