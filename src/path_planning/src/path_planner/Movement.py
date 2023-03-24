# Name: Movement Class Definition
# Authors: Conor Nichols (cjnichols1@sheffield.ac.uk) Joseph Fields (jfields1@sheffield.ac.uk)

import rospy

from geometry_msgs.msg import Pose
#import matplotlib.pyplot as plt

class Movement:
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper
    
    def move(self, pos:Pose, final_link_name=""):
        """ Safely move to desired position using IK, checking robot will stay within zone
        INPUT: Pose pos
        OUTPUT: bool Success - Returns True is movement succesful, False if not possible or failed.
        """
        SF = 100 #distance scale factor
        Q = [12,12,10,8,6,4,2] #'size' of the object
        D = 10
        PathComplete=0
        # Get block coordinates relative to robot instead of world
        pos_robot_base_frame = self.serv_helper.frameConverter((self.serv_helper.robot_ns+"_base"), "world", pos)
        ##Goal position
        xgoal = pos_robot_base_frame.position.x*SF 
        ygoal = pos_robot_base_frame.position.y*SF
        zgoal = pos_robot_base_frame.position.z*SF +1
        print("startxyz->goalxyz:",startx,starty,startz,xgoal,ygoal,zgoal)
        ##Start position relative to world then arm
        start_pose_world=self.serv_helper.getLinkPos(self.serv_helper.robot_ns,"link6") 
        start_pose = self.serv_helper.frameConverter((self.serv_helper.robot_ns+"_base"), "world", start_pose_world)
        startx = start_pose.position.x*SF #start coords for end effector (now relative)
        starty = start_pose.position.y*SF
        startz = start_pose.position.z*SF
        
        while PathComplete==0:
            #Obstacle positions relative to world then arm
            robot_namespaces = ["mover6_a", "mover6_b"]
            xobj=[]
            yobj=[]
            zobj=[]
            robot_namespaces.remove(self.serv_helper.robot_ns)
            for obstacle_arm_ns in robot_namespaces:
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
            FilterPathTakenLen=30 #approximately, configurable constant
            SamplePeriod=round(PathTakenLen/FilterPathTakenLen) #sample period
            FilterPathTakenx=[PathTakenx[0]]
            FilterPathTakeny=[PathTakeny[0]]
            FilterPathTakenz=[PathTakenz[0]]

            if PathTakenLen>FilterPathTakenLen:
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
            tempPos.position.x=FilterPathTakenx[1]
            tempPos.position.y=FilterPathTakeny[1]
            tempPos.position.z=FilterPathTakenz[1]
            tempPos.orientation.x= pos_robot_base_frame.orientation.x 
            tempPos.orientation.y= pos_robot_base_frame.orientation.y 
            tempPos.orientation.z= pos_robot_base_frame.orientation.z 
            tempPos.orientation.w= pos_robot_base_frame.orientation.w

            rospy.loginfo("Path Planner - Move - Publishing %s to\t%.2f\t%.2f\t%.2f\t\t%.2f\t%.2f\t%.2f\t%.2f", self.serv_helper.robot_ns, tempPos.position.x, tempPos.position.y, tempPos.position.z, tempPos.orientation.x, tempPos.orientation.y, tempPos.orientation.z, tempPos.orientation.w)
            d=self.serv_helper.EuclidianDistance(tempPos.position.x,tempPos.position.y,tempPos.position.z,xgoal,ygoal,zgoal)
            if d <= 5:
                precise_angle_flag=1 #orientation does matter - small tolerance
            else:
                precise_angle_flag=0 #orientation does not matter - wide tolerance

            # Move robot to new position, in robot reference frame
            status = self.serv_helper.move(tempPos, final_link_name,precise_angle_flag)
            #TODO: Force wait until robot has reached desired position. Temp fix:
            rospy.sleep(0.1)
            if not(status):
                rospy.logerr("Path Planner - Error, Target position unreachable.")

            if d <= 2:
                PathComplete = 1
            else:
                PathComplete = 0
        
        return status #TODO: Implement zone checks
