# Name: Movement Class Definition
# Authors: Conor Nichols (cjnichols1@sheffield.ac.uk) Joseph Fields (jfields1@sheffield.ac.uk)

import rospy

from geometry_msgs.msg import Pose
#import matplotlib.pyplot as plt

class Movement:
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper
    
    def move(self, pos:Pose):
        """ Safely move to desired position using IK, checking robot will stay within zone
        INPUT: Pose pos
        OUTPUT: bool Success - Returns True is movement succesful, False if not possible or failed.
        """
        SF=100 #distance scale factor
        ##Start position
        start_pose=self.serv_helper.getJointPos(self.serv_helper.robot_ns,self.serv_helper.robot_ns,"/link6")
        startx = start_pose.position.x*SF #start position for arm (now relative)
        starty = start_pose.position.y*SF
        startz = start_pose.position.z*SF
        
        # Get coordinates relative to robot instead of world
        pos_robot_base_frame = self.serv_helper.frameConverter((self.serv_helper.robot_ns+"_base"), "world", pos)
        
        #print("Path Planner - Move - Publishing ", self.serv_helper.robot_ns, " to ", pos_robot_frame.position.x, "\t", pos_robot_frame.position.y, "\t", pos_robot_frame.position.z, "\t", pos_robot_frame.orientation.x, "\t", pos_robot_frame.orientation.y, "\t", pos_robot_frame.orientation.z, "\t", pos_robot_frame.orientation.w)

        ##Goal position
        xgoal = pos_robot_base_frame.position.x*SF 
        ygoal = pos_robot_base_frame.position.y*SF
        zgoal = pos_robot_base_frame.position.z*SF
        #print("startxy->goalxy:",startx,starty,xgoal,ygoal)
        
        #Obstacle positions
        if self.serv_helper.robot_ns=="mover6_a":
            obstacle_arm_ns="mover6_b"
        elif self.serv_helper.robot_ns=="mover6_b":
            obstacle_arm_ns="mover6_a"
        else:
            rospy.logerr("ERROR-Invalid Robot name")
        
        xobj=[]
        yobj=[]
        zobj=[]
        for obs in range(0,7):
            if obs==0:
              obs_link="/base_link"
            else:
                obs_link="/link"+str(obs)
            pos_obstacle=self.serv_helper.getJointPos(self.serv_helper.robot_ns,obstacle_arm_ns,obs_link)
            xobj.append(pos_obstacle.position.x *SF)
            yobj.append(pos_obstacle.position.y *SF)
            zobj.append(pos_obstacle.position.z *SF)
        #print(xobj,yobj)


        Q = [12,4,2]
        D = 10
        xobj,yobj,zobj,Q = Link_Midpoints(xobj,yobj,zobj,Q)
        #print(xobj,yobj,Q)
        ##Visual Commands
        X, Y,Z, xline, yline,zline, PotentialEnergy, EnergyPathTaken, PathTaken = Space_Generation(startx, starty,startz,xgoal, ygoal,zgoal, xobj, yobj,zobj, Q, D)
        plotAPF(xobj, yobj,zobj, xline, yline,zline, PotentialEnergy, EnergyPathTaken)
        #plotPath(PathTaken)
        ##X,Y path the End effector will take
        PathTaken = PathPlanner(startx,starty,startz,xgoal, ygoal,zgoal, xobj, yobj,zobj, Q, D)
        #print(len(PathTaken))
        PathTaken = [[x[0]/SF,x[1]/SF] for x in PathTakenSF]
        #print(len(PathTaken))

        ##Visual Commands
        #X, Y, xline, yline, PotentialEnergy, EnergyPathTaken = self.serv_helper.Space_Generation(PathTakenSF, xgoal, ygoal, xobj, yobj, Q, D)
        #self.serv_helper.plotAPF(X, Y, xline, yline, PotentialEnergy, EnergyPathTaken)
        #self.serv_helper.plotPath(PathTakenSF)

        tempPos=Pose()
        for incr in range(len(PathTaken)): #move incrementally through positions
            #rospy.loginfo("move to: %s",PathTaken[incr])
            incrx,incry=PathTaken[incr]
            tempPos.position.x=incrx
            tempPos.position.y=incry
            tempPos.position.z= pos_robot_base_frame.position.z #temporary
            tempPos.orientation.x= pos_robot_base_frame.orientation.x #temporary
            tempPos.orientation.y= pos_robot_base_frame.orientation.y #temporary
            tempPos.orientation.z= pos_robot_base_frame.orientation.z #temporary
            tempPos.orientation.w= pos_robot_base_frame.orientation.w
            #self.serv_helper.move(tempPos)

            #TODO: Force wait until robot has reached desired position. Temp fix:

        return True
