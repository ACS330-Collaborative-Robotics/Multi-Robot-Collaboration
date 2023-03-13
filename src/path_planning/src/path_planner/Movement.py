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
        ##Start position relative to world then arm
        start_pose_world=self.serv_helper.getLinkPos(self.serv_helper.robot_ns,"link6") 
        start_pose = self.serv_helper.frameConverter((self.serv_helper.robot_ns+"_base"), "world", start_pose_world)
        startx = start_pose.position.x*SF #start coords for end effector (now relative)
        starty = start_pose.position.y*SF
        startz = start_pose.position.z*SF
        
        # Get block coordinates relative to robot instead of world
        pos_robot_base_frame = self.serv_helper.frameConverter((self.serv_helper.robot_ns+"_base"), "world", pos)

        ##Goal position
        xgoal = pos_robot_base_frame.position.x*SF 
        ygoal = pos_robot_base_frame.position.y*SF
        zgoal = pos_robot_base_frame.position.z*SF +1
        print("startxyz->goalxyz:",startx,starty,startz,xgoal,ygoal,zgoal)
        
        #Obstacle positions relative to world then arm

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
              obs_link="base_link"
            else:
                obs_link="link"+str(obs)

            pos_obstacle_world=self.serv_helper.getLinkPos(obstacle_arm_ns,obs_link) #obstacle arm joint positions relative to world
            pos_obstacle = self.serv_helper.frameConverter((self.serv_helper.robot_ns+"_base"), "world", pos_obstacle_world)
            xobj.append(pos_obstacle.position.x *SF) #obstacle arm joint positions relative to other arm
            yobj.append(pos_obstacle.position.y *SF)
            zobj.append(pos_obstacle.position.z *SF)
        #print(len(xobj),len(yobj),len(zobj))


        Q = [12,12,10,8,6,4,2] #strength of field around each arm joint?
        D = 10
        xobj,yobj,zobj,Q = self.serv_helper.Link_Midpoints(xobj,yobj,zobj,Q) #????

        ##Visual Commands
        #X,Y,Z, xline, yline,zline, PotentialEnergy, EnergyPathTaken, PathTaken = self.serv_helper.Space_Generation(startx, starty,startz,xgoal, ygoal,zgoal, xobj, yobj,zobj, Q, D)
        #self.serv_helper.plotAPF(xobj, yobj,zobj, xline, yline,zline, PotentialEnergy, EnergyPathTaken)
        #self.serv_helper.plotPath(PathTaken)

        ##X,Y,Z path the End effector will take
        PathTakenSFx, PathTakenSFy, PathTakenSFz = self.serv_helper.PathPlanner(startx,starty,startz,xgoal,ygoal,zgoal,xobj,yobj,zobj, Q, D)
        PathTakenx = [x/SF for x in PathTakenSFx] #rescale back to meters
        PathTakeny = [y/SF for y in PathTakenSFy]
        PathTakenz = [z/SF for z in PathTakenSFz]
        
        ##FILTERING
        PathTakenLen=len(PathTakenx)
        FilterPathTakenLen=20 #approximately, configurable constant
        SamplePeriod=round(PathTakenLen/FilterPathTakenLen) #sample period
        FilterPathTakenx=[]
        FilterPathTakeny=[]
        FilterPathTakenz=[]
        if PathTakenLen>FilterPathTakenLen:
            j=0
            for i in range(PathTakenLen):
                if i%SamplePeriod == 0: #take a sample every SamplePeriod iterations
                    FilterPathTakenx.append(PathTakenx[i])
                    FilterPathTakeny.append(PathTakeny[i])
                    FilterPathTakenz.append(PathTakenz[i])
            print("FILTERED PATH LENGTH=",len(FilterPathTakenx))
        else:
            FilterPathTakenx=PathTakenx
            FilterPathTakeny=PathTakeny
            FilterPathTakenz=PathTakenz

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
