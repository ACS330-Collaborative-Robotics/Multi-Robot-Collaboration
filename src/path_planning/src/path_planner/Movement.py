# Name: Movement Class Definition
# Authors: Conor Nichols (cjnichols1@sheffield.ac.uk) Joseph Fields (jfields1@sheffield.ac.uk)

import rospy
from pathlib import Path
from geometry_msgs.msg import Pose
from time import time

#import matplotlib.pyplot as plt

import yaml
from yaml.loader import SafeLoader
class Movement:
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper

        config_file_name = str(Path.home()) + '/catkin_ws/src/path_planning/config/settings.yaml'
        
        with open(config_file_name) as yamlfile: # add right path
            self.serv_helper.APFyamlData = yaml.load(yamlfile, Loader=SafeLoader)
        print(self.serv_helper.APFyamlData)
    
    def move(self, pos:Pose, final_link_name=""):
        """ Safely move to desired position using IK, checking robot will stay within zone
        INPUT: Pose pos
        OUTPUT: bool Success - Returns True is movement succesful, False if not possible or failed.
        """
        SF = 100 #distance scale factor
        Q = [10,10,10,10,10,10] #'size' of the object #TODO(WILL CAUSE ISSUES WITH MORE ROBOTS)
        D = self.serv_helper.APFyamlData["D"]
        PathComplete=0
        robot_namespaces = ["mover6_a", "mover6_b"] #TODO: will be changed to a service to get names of connected arms
        # Get block coordinates relative to robot instead of world
        pos_robot_base_frame = self.serv_helper.frameConverter((self.serv_helper.robot_ns+"/base_link"), "world", pos)
        ##Goal position
        xgoal = pos_robot_base_frame.position.x*SF 
        ygoal = pos_robot_base_frame.position.y*SF
        zgoal = pos_robot_base_frame.position.z*SF 
        
        ##Start position relative to world then arm
        start_pose_world=self.serv_helper.getLinkPos(self.serv_helper.robot_ns,"link6") 
        start_pose = self.serv_helper.frameConverter((self.serv_helper.robot_ns+"/base_link"), "world", start_pose_world)
        startx = start_pose.position.x*SF #start coords for end effector (now relative)
        starty = start_pose.position.y*SF
        startz = start_pose.position.z*SF
        #print("startxyz->goalxyz:",startx,starty,startz,xgoal,ygoal,zgoal)
        while PathComplete==0 and not rospy.is_shutdown():
            start_time = time()
            #Obstacle positions relative to world then arm
            robot_namespaces = ["mover6_a", "mover6_b"] #TODO: will be changed to a service to get names of connected arms
            xobj=[0, 0, 0, 0, 0, 0]
            yobj=[0, 0, 0, 0, 0, 0]
            zobj=[0, 5, 10, 15, 20, 25]
            tempxobj = []
            tempyobj = []
            tempzobj = []
            tempQ = []
            robot_namespaces.remove(self.serv_helper.robot_ns) #remove own name from list of arms to avoid
            for obstacle_arm_ns in robot_namespaces:
                for obs in range(0,7):
                    if obs==0:
                        obs_link="base_link"
                    else:
                        obs_link="link"+str(obs)

                    pos_obstacle_world=self.serv_helper.getLinkPos(obstacle_arm_ns,obs_link) #obstacle arm joint positions relative to world
                    pos_obstacle = self.serv_helper.frameConverter((self.serv_helper.robot_ns+"/base_link"), "world", pos_obstacle_world)
                    xobj.append(pos_obstacle.position.x *SF) #obstacle arm joint positions relative to other arm
                    yobj.append(pos_obstacle.position.y *SF)
                    zobj.append(pos_obstacle.position.z *SF)
                #print(len(xobj),len(yobj),len(zobj))

            #xobj,yobj,zobj,Q = self.serv_helper.Link_Midpoints(xobj,yobj,zobj,Q) #turns joint objects into a line of objects along link
            #xobj.append(0) #own base as an object 
            #yobj.append(0)
            #zobj.append(0)
            #Q.append(0.1)
            ##Visual Commands
            #X,Y,Z, xline, yline,zline, PotentialEnergy, EnergyPathTaken, PathTaken = self.serv_helper.Space_Generation(startx, starty,startz,xgoal, ygoal,zgoal, xobj, yobj,zobj, Q, D)
            #self.serv_helper.plotAPF(xobj, yobj,zobj, xline, yline,zline, PotentialEnergy, EnergyPathTaken)
            #self.serv_helper.plotPath(PathTaken)

            ##X,Y,Z path the End effector will take
            PathTakenSFx, PathTakenSFy, PathTakenSFz, Objectx, Objecyy, Objectz, ObjectQ = self.serv_helper.PathPlanner(startx,starty,startz,xgoal,ygoal,zgoal,xobj,yobj,zobj, Q, D,tempxobj,tempyobj,tempzobj,tempQ)
            PathTakenx = [x/SF for x in PathTakenSFx] #rescale back to meters
            PathTakeny = [y/SF for y in PathTakenSFy]
            PathTakenz = [z/SF for z in PathTakenSFz]
            
            ##FILTERING
            #PathTakenLen=len(PathTakenx)
            #FilterPathTakenLen=30 #approximate filtered length, configurable constant
            #SamplePeriod=round(PathTakenLen/FilterPathTakenLen) #sample period

            #FilterPathTakenx=[PathTakenx[0]]
            #FilterPathTakeny=[PathTakeny[0]]
            #FilterPathTakenz=[PathTakenz[0]]
            #if PathTakenLen>FilterPathTakenLen: #if too many path points, reduce
             #   for i in range(PathTakenLen):
             #       if i%SamplePeriod == 0: #take a sample every SamplePeriod iterations
             #           FilterPathTakenx.append(PathTakenx[i])
             #           FilterPathTakeny.append(PathTakeny[i])
             #           FilterPathTakenz.append(PathTakenz[i])
             #   rospy.loginfo("FILTERED PATH LENGTH= %d",len(FilterPathTakenx))
            #else: #if fewer than 30 path points, just use produced path
             #   FilterPathTakenx=PathTakenx
             #   FilterPathTakeny=PathTakeny
             #   FilterPathTakenz=PathTakenz

            arm_pos=Pose() #pose for next coordinate
            arm_pos.position.x=PathTakenx[1]
            arm_pos.position.y=PathTakeny[1]
            arm_pos.position.z=PathTakenz[1]
            arm_pos.orientation.x= pos_robot_base_frame.orientation.x 
            arm_pos.orientation.y= pos_robot_base_frame.orientation.y 
            arm_pos.orientation.z= pos_robot_base_frame.orientation.z 
            arm_pos.orientation.w= pos_robot_base_frame.orientation.w

            #rospy.loginfo("Path Planner - Move - Publishing %s to\t%.2f\t%.2f\t%.2f\t\t%.2f\t%.2f\t%.2f\t%.2f", self.serv_helper.robot_ns, arm_pos.position.x, arm_pos.position.y, arm_pos.position.z, arm_pos.orientation.x, arm_pos.orientation.y, arm_pos.orientation.z, arm_pos.orientation.w)
            d=self.serv_helper.EuclidianDistance(arm_pos.position.x*SF,arm_pos.position.y*SF,arm_pos.position.z*SF,xgoal,ygoal,zgoal)
            if d <= 3: #when close, use precise orientation
                precise_angle_flag=1 #orientation does matter - small tolerance
            else:
                precise_angle_flag=0 #orientation does not matter - wide tolerance

            rospy.loginfo("Potential Fields - Step Calculation Time: %.4f",time()-start_time)
            # Move robot to new position, in robot reference frame
            status = self.serv_helper.move(arm_pos, final_link_name,precise_angle_flag)
            #TODO: Force wait until robot has reached desired position. Temp fix:
            rospy.sleep(0.1)
            if not(status):
                #rospy.logerr("Path Planner - Error, Target position unreachable.")
                pass
            else: #check if movement ran
                if d <= 2:
                    PathComplete = 1
                else:
                    startx = arm_pos.position.x*SF #start coords for end effector (now next step)
                    starty = arm_pos.position.y*SF 
                    startz = arm_pos.position.z*SF
            rospy.loginfo('New Position - %.3f %.3f %.3f',startx,starty,startz)
        return status #TODO: Implement zone checks


