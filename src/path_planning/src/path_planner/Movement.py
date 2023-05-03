# Name: Movement Class Definition
# Authors: Conor Nichols (cjnichols1@sheffield.ac.uk) Joseph Fields (jfields1@sheffield.ac.uk)

import rospy
from pathlib import Path
from geometry_msgs.msg import Pose
from time import time
import tf_conversions

import yaml
from yaml.loader import SafeLoader
class Movement:
    def __init__(self, serv_helper):
        self.serv_helper = serv_helper

        config_file_name = str(Path.home()) + '/catkin_ws/src/path_planning/config/settings.yaml'
        
        with open(config_file_name) as yamlfile: # add right path
            self.serv_helper.APFyamlData = yaml.load(yamlfile, Loader=SafeLoader)
        #print(self.serv_helper.APFyamlData)
    
    def move(self, pos:Pose, allow_imprecise_orientation:bool, final_link_name=""):
        """ Safely move to desired position using IK, checking robot will stay within zone
        INPUT: Pose pos
        OUTPUT: bool Success - Returns True is movement succesful, False if not possible or failed.
        """
        precise_angle_flag = 0
        SF = 100 #distance scale factor
        D = self.serv_helper.APFyamlData["D"]
        CloseEnough = 0.8
        PathComplete = 0
        robot_namespaces = ["mover6_a", "mover6_b"] #TODO: will be changed to a service to get names of connected arms
        # Get block coordinates relative to robot instead of world
        
        pos_robot_base_frame = self.serv_helper.frameConverter((self.serv_helper.robot_ns+"/base_link"), "world", pos)
        ##Goal position
        xgoal = pos_robot_base_frame.position.x*SF 
        ygoal = pos_robot_base_frame.position.y*SF
        zgoal = pos_robot_base_frame.position.z*SF 
        
        ##Start position relative to world then arm base
        start_pose_world=self.serv_helper.getLinkPos(self.serv_helper.robot_ns,"link6") 
        start_pose = self.serv_helper.frameConverter((self.serv_helper.robot_ns+"/base_link"), "world", start_pose_world)
        startx = start_pose.position.x*SF #start coords for end effector (now relative)
        starty = start_pose.position.y*SF
        startz = start_pose.position.z*SF
        #print("startxyz->goalxyz:",startx,starty,startz,xgoal,ygoal,zgoal)
        while PathComplete == 0 and not rospy.is_shutdown():
            #rospy.logerr("Target World: %.2f %.2f %.2f, Target base: %.1f %.1f %.1f",pos.position.x, pos.position.y, pos.position.z, xgoal,ygoal,zgoal)
            #rospy.logerr("Arm World: %.2f %.2f %.2f, Arm base: %.1f %.1f %.1f",start_pose_world.position.x, start_pose_world.position.y, start_pose_world.position.z, startx,starty,startz)
            start_time = time()
            #Obstacle positions relative to world then arm
            robot_namespaces = ["mover6_a", "mover6_b"] #TODO: will be changed to a service to get names of connected arms

            xobj = [0,0,0]
            yobj = [0,0,0]
            zobj = [0,18,50]
            Q = [9,21,27] #'size' of the object #TODO(WILL CAUSE ISSUES WITH MORE ROBOTS)
            xobj,yobj,zobj,Q = self.serv_helper.Link_Midpoints(xobj,yobj,zobj,Q)
            tempxobj = []
            tempyobj = []
            tempzobj = []
            tempQ = []
            tempxobj_linked = []
            tempyobj_linked = []
            tempzobj_linked = []
            tempQ_linked = []

            robot_namespaces.remove(self.serv_helper.robot_ns) #remove own name from list of arms to avoid
            for obstacle_arm_ns in robot_namespaces:
                for obs in range(0,10):
                    if obs == 0:
                        obs_link = "base_link"
                    elif obs == 8:
                        obs_link = "gripper_a"
                    elif obs == 9:
                        obs_link = "gripper_b"
                    else:
                        obs_link = "link"+str(obs)

                    pos_obstacle_world=self.serv_helper.getLinkPos(obstacle_arm_ns,obs_link) #obstacle arm joint positions relative to world
                    if pos_obstacle_world == None:
                        rospy.logfatal("Path Planner - getLinkPos")
                    else:
                        pos_obstacle = self.serv_helper.frameConverter((self.serv_helper.robot_ns+"/base_link"), "world", pos_obstacle_world)

                        tempxobj.append(pos_obstacle.position.x * SF) #obstacle arm joint positions relative to other arm
                        tempyobj.append(pos_obstacle.position.y * SF)
                        tempzobj.append(pos_obstacle.position.z * SF)
                        tempQ.append(12)

                #xobj,yobj,zobj,Q = self.serv_helper.Link_Midpoints(xobj,yobj,zobj,Q) #turns joint objects into a line of objects along link
                tempxobj_linked,tempyobj_linked,tempzobj_linked,tempQ_linked = self.serv_helper.Link_Midpoints(tempxobj,tempyobj,tempzobj,tempQ)

                #append into xobjs here
                xobj = xobj + tempxobj_linked
                yobj = yobj + tempyobj_linked
                zobj = zobj + tempzobj_linked
                Q = Q + tempQ_linked

            #if self.serv_helper.is_obsarm_in_zone(robot_namespaces ,pos.position.x,pos.position.y): #working in world frame
             #   xobj = xobj + [xgoal]
             #   yobj = yobj + [ygoal]
             #   zobj = zobj + [0]
             #   Q = Q + ([18])
             #   rospy.logwarn("Path Planner - Forcefield activated to repel %s",self.serv_helper.robot_ns)

            ##X,Y,Z path the End effector will take
            X, Y, Z, = self.serv_helper.PathPlanner(startx,starty,startz,xgoal,ygoal,zgoal,xobj,yobj,zobj, Q, D,tempxobj,tempyobj,tempzobj,tempQ,precise_angle_flag)
              #rescale back to meters

            arm_pos = Pose() #pose for next coordinate
            arm_pos.position.x = X/SF
            arm_pos.position.y = Y/SF
            arm_pos.position.z = Z/SF

            arm_pos.orientation.x = pos_robot_base_frame.orientation.x 
            arm_pos.orientation.y = pos_robot_base_frame.orientation.y 
            arm_pos.orientation.z = pos_robot_base_frame.orientation.z 
            arm_pos.orientation.w = pos_robot_base_frame.orientation.w
            #rospy.loginfo('Current: %.2f, %.2f, %.2f, Goal: %.2f, %.2f, %.2f,',arm_pos.position.x*SF,arm_pos.position.y*SF,arm_pos.position.z*SF,xgoal,ygoal,zgoal)
            d = self.serv_helper.EuclidianDistance(arm_pos.position.x*SF,arm_pos.position.y*SF,arm_pos.position.z*SF,xgoal,ygoal,zgoal)
            rospy.logdebug('Distance: %.2f', d)
            if allow_imprecise_orientation and d > 5:
                precise_angle_flag = 0
            else:
                precise_angle_flag = 1

            if arm_pos.position.z < 0.05:
                arm_pos.position.z = 0.05
                
            rospy.logdebug("Path Planner - Moving %s for %s to X: %.2f Y: %.2f Z: %.2f x: %.1f y: %.1fz: %.1f w: %.1f Precision: %s", self.serv_helper.robot_ns,self.serv_helper.target_block, arm_pos.position.x, arm_pos.position.y, arm_pos.position.z, arm_pos.orientation.x, arm_pos.orientation.y, arm_pos.orientation.z, arm_pos.orientation.w,bool(precise_angle_flag))

            #rospy.loginfo("Potential Fields - Step Calculation Time: %.4f",time()-start_time)
            #deltax =  startx - arm_pos.position.x*SF
            #deltay =  starty - arm_pos.position.y*SF
            #deltaz =  startz - arm_pos.position.z*SF
            #rospy.loginfo("deltas: %.2f %.2f %.2f",deltax,deltay,deltaz)
            # Move robot to new position, in robot reference frame

            #print("\n\n\n I AM MOVING THE ARM \n\n\n")
            status = self.serv_helper.move(arm_pos, final_link_name,precise_angle_flag)
            #TODO: Force wait until robot has reached desired position. Temp fix:
            rospy.sleep(0.1)

            #if precise_angle_flag:
            #    CloseEnough = 0.5
            #else:
            #    CloseEnough = 5
            if not(status):
                #rospy.logerr("Path Planner - Error, Target position unreachable.")
                pass
            else: #check if movement ran
                if d <= CloseEnough:
                    PathComplete = 1
                else:
                    startx = arm_pos.position.x*SF #start coords for end effector (now next step)
                    starty = arm_pos.position.y*SF 
                    startz = arm_pos.position.z*SF
            #rospy.loginfo('New Position - %.3f %.3f %.3f',startx,starty,startz)
        return status #TODO: Implement zone checks


