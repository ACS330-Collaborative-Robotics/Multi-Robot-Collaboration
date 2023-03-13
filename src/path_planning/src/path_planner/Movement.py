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
        zgoal = pos_robot_base_frame.position.z*SF +1
        print("startxyz->goalxyz:",startx,starty,startz,xgoal,ygoal,zgoal)
        
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
        #print(len(xobj),len(yobj),len(zobj))


        Q = [12,12,10,8,6,4,2] #strength of field around each point?
        D = 10
        xobj,yobj,zobj,Q = self.serv_helper.Link_Midpoints(xobj,yobj,zobj,Q)

        ##Visual Commands
        #X,Y,Z, xline, yline,zline, PotentialEnergy, EnergyPathTaken, PathTaken = self.serv_helper.Space_Generation(startx, starty,startz,xgoal, ygoal,zgoal, xobj, yobj,zobj, Q, D)
        #self.serv_helper.plotAPF(xobj, yobj,zobj, xline, yline,zline, PotentialEnergy, EnergyPathTaken)
        #self.serv_helper.plotPath(PathTaken)

        ##X,Y,Z path the End effector will take
        PathTakenSFx, PathTakenSFy, PathTakenSFz = self.serv_helper.PathPlanner(startx,starty,startz,xgoal,ygoal,zgoal,xobj,yobj,zobj, Q, D)
        PathTakenx = [x/SF for x in PathTakenSFx]
        PathTakeny = [y/SF for y in PathTakenSFy]
        PathTakenz = [z/SF for z in PathTakenSFz]
        

        tempPos=Pose()
        for incr in range(len(PathTakenx)): #move incrementally through positions
            #rospy.loginfo("move to: %s",PathTaken[incr])
            tempPos.position.x=PathTakenx[incr]
            tempPos.position.y=PathTakeny[incr]
            tempPos.position.z=PathTakenz[incr]
            tempPos.orientation.x= pos_robot_base_frame.orientation.x #temporary
            tempPos.orientation.y= pos_robot_base_frame.orientation.y #temporary
            tempPos.orientation.z= pos_robot_base_frame.orientation.z #temporary
            tempPos.orientation.w= pos_robot_base_frame.orientation.w
            self.serv_helper.move(tempPos)

            #TODO: Force wait until robot has reached desired position. Temp fix:

        return True
