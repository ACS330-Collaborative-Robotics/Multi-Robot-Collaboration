# Name: Movement Class Definition
# Authors: Conor Nichols (cjnichols1@sheffield.ac.uk) Joseph Fields (jfields1@sheffield.ac.uk)

import rospy
import tf_conversions

from geometry_msgs.msg import Pose

class Movement:
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper
    
    def move(self, pos:Pose):
        """ Safely move to desired position using IK, checking robot will stay within zone

        INPUT: Pose pos
        OUTPUT: bool Success - Returns True is movement succesful, False if not possible or failed.
        """

        # Get coordinates relative to robot instead of world
        pos_robot_frame = self.serv_helper.frameConverter(self.serv_helper.robot_ns, "world", pos)

        # Convert to Euler angles as IK service uses them
        euler_angles = tf_conversions.transformations.euler_from_quaternion([pos_robot_frame.orientation.x, pos_robot_frame.orientation.y, pos_robot_frame.orientation.z, pos_robot_frame.orientation.w])

        pos_robot_frame.orientation.x = euler_angles[0]
        pos_robot_frame.orientation.y = euler_angles[1]
        pos_robot_frame.orientation.z = euler_angles[2]
        pos_robot_frame.orientation.w = 0

        # Move robot to new position, in robot reference frame
        self.serv_helper.move(pos_robot_frame)
        
        #TODO: Force wait until robot has reached desired position. Temp fix:
        rospy.sleep(3)

        return True #TODO: Implement zone checks

    def APFmove(self, pos:Pose):
        """ Safely move to desired position using IK, checking robot will stay within zone
        INPUT: Pose pos
        OUTPUT: bool Success - Returns True is movement succesful, False if not possible or failed.
        """
        
        start_pose=self.serv_helper.getJointPos(self.serv_helper.robot_ns,self.serv_helper.robot_ns,6)
        startx = start_pose.position.x #start position for arm (now relative)
        starty = start_pose.position.y

        pos_robot_frame = self.serv_helper.frameConverter(self.serv_helper.robot_ns, "world", pos) #get desired pose in robot frame
        
        euler_angles = tf_conversions.transformations.euler_from_quaternion([pos_robot_frame.orientation.x, pos_robot_frame.orientation.y, pos_robot_frame.orientation.z, pos_robot_frame.orientation.w])
        pos_robot_frame.orientation.x = euler_angles[0]
        pos_robot_frame.orientation.y = euler_angles[1]
        pos_robot_frame.orientation.z = euler_angles[2]
        pos_robot_frame.orientation.w = 0 #currently no angle stuff

        xgoal = pos_robot_frame.position.x #position of goal x
        ygoal = pos_robot_frame.position.y

        xobj = [0.5] #obstacles will add for loop to look at other arm
        yobj = [0.5]
        Q = 0.01
        D = 0.05
        ##Visual Commands
        X, Y, xline, yline, PotentialEnergy, EnergyPathTaken = self.serv_helper.APFSpace_Generation(startx, starty, xgoal, ygoal, xobj, yobj, Q, D) 
        self.serv_helper.APFplot(X, Y, xline, yline, PotentialEnergy, EnergyPathTaken)

        ##X,Y path the End effector will take
        rospy.loginfo([startx,starty,xgoal,ygoal,xobj,yobj,Q,D])
        PathTaken = self.serv_helper.APFPathPlanner(startx,starty,xgoal,ygoal,xobj,yobj,Q,D)
        rospy.loginfo(PathTaken)
        ## add a while loop to move through the points?
        tempPos=Pose()
        for incr in range(len(PathTaken)): #move incrementally through positions
            rospy.loginfo("move to incr %s",PathTaken[incr])
            incrx,incry=PathTaken[incr]
            tempPos.position.x=incrx
            tempPos.position.y=incry
            tempPos.position.z= pos_robot_frame.position.z #temporary
            tempPos.orientation.x= pos_robot_frame.orientation.x #temporary
            tempPos.orientation.y= pos_robot_frame.orientation.y #temporary
            tempPos.orientation.z= pos_robot_frame.orientation.z #temporary
            self.serv_helper.move(tempPos)

            #TODO: Force wait until robot has reached desired position. Temp fix:
            rospy.sleep(0.01)

        return True
