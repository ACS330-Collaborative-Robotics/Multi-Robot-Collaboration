import rospy

from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState

def talker():
    rospy.init_node('object_mover')

    rospy.wait_for_service('gazebo/get_model_state')
    get_model_state_serv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

    rospy.wait_for_service('gazebo/get_model_state')
    set_model_state_serv = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
    

    model_name = "block9" #TODO: Make this work for robots
    reference_frame = "world"
    data = get_model_state_serv(model_name, reference_frame)
    print(data)

    # Setup object to be updated and sent back
    temp = ModelState()
    temp.model_name = model_name
    temp.pose = data.pose
    temp.twist = data.twist
    temp.reference_frame = reference_frame

    # Make changes
    temp.pose.position.y = -0.3

    print(temp)

    print(set_model_state_serv(temp))


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass