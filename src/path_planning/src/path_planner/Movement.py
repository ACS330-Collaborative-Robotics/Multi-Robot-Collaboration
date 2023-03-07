# Name: Movement Class Definition
# Authors: Conor Nichols (cjnichols1@sheffield.ac.uk) Joseph Fields (jfields1@sheffield.ac.uk)

import rospy

from geometry_msgs.msg import Pose

class Movement:
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper
    
    def move(self, pos:Pose):
        """ Safely move to desired position using IK, checking robot will stay within zone
        INPUT: Pose pos
        OUTPUT: bool Success - Returns True is movement succesful, False if not possible or failed.
        """
        SF=100
        start_pose=self.serv_helper.getJointPos(self.serv_helper.robot_ns,self.serv_helper.robot_ns,6)
        startx = start_pose.position.x*SF #start position for arm (now relative)
        starty = start_pose.position.y*SF

        # Get coordinates relative to robot instead of world
        pos_robot_frame = self.serv_helper.frameConverter(self.serv_helper.robot_ns, "world", pos)

        print("Path Planner - Move - Publishing ", self.serv_helper.robot_ns, " to ", pos_robot_frame.position.x, "\t", pos_robot_frame.position.y, "\t", pos_robot_frame.position.z, "\t", pos_robot_frame.orientation.x, "\t", pos_robot_frame.orientation.y, "\t", pos_robot_frame.orientation.z, "\t", pos_robot_frame.orientation.w)

        xgoal = pos_robot_frame.position.x*SF #position of goal x
        ygoal = pos_robot_frame.position.y*SF

        xobj = [50] #obstacles will add for loop to look at other arm
        yobj = [50]
        Q = 15
        D = 10

        ##Visual Commands
        #X, Y, xline, yline, PotentialEnergy, EnergyPathTaken, PathTakenSF = self.serv_helper.Space_Generation(startx, starty, xgoal, ygoal, xobj, yobj, Q, D)
        #self.serv_helper.plotAPF(X, Y, xline, yline, PotentialEnergy, EnergyPathTaken)
        #self.serv_helper.plotPath(PathTakenSF)
        ##X,Y path the End effector will take
        PathTakenSF = self.serv_helper.PathPlanner(startx,starty,xgoal, ygoal, xobj, yobj, Q, D)
        
        PathTaken = [[x[0]/SF,x[1]/SF] for x in PathTakenSF]

        print(len(PathTaken))
        
        ## add a while loop to move through the points?
        tempPos=Pose()
        for incr in range(len(PathTaken)): #move incrementally through positions
            rospy.loginfo("move to: %s",PathTaken[incr])
            incrx,incry=PathTaken[incr]
            tempPos.position.x=incrx
            tempPos.position.y=incry
            tempPos.position.z= pos_robot_frame.position.z #temporary
            tempPos.orientation.x= pos_robot_frame.orientation.x #temporary
            tempPos.orientation.y= pos_robot_frame.orientation.y #temporary
            tempPos.orientation.z= pos_robot_frame.orientation.z #temporary
            tempPos.orientation.w= pos_robot_frame.orientation.w
            self.serv_helper.move(tempPos)

            #TODO: Force wait until robot has reached desired position. Temp fix:
            rospy.sleep(0.00000001)

        return True
