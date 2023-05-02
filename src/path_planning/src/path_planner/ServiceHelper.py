# Name: ServiceHelper Class Definition
# Authors: Conor Nichols (cjnichols1@sheffield.ac.uk) Joseph Fields (jfields1@sheffield.ac.uk)

import rospy
import tf2_ros
import tf
from pathlib import Path
import tf_conversions

from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetLinkState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from tf2_geometry_msgs import PoseStamped
from inv_kinematics.srv import InvKin
from inv_kinematics.srv import InvKinRequest
from std_msgs.msg import Header
from std_msgs.msg import Bool


#from APF dependancies
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import yaml
from yaml.loader import SafeLoader

import math
from math import *

class ServiceHelper:
    def __init__(self, robot_ns,target_block):
        self.robot_ns = robot_ns
        self.target_block = target_block
        # Setup inverse_kinematics service
        rospy.wait_for_service('/inverse_kinematics')
        self.inv_kin = rospy.ServiceProxy('/inverse_kinematics', InvKin)

        rospy.wait_for_service('/inverse_kinematics_reachability')
        self.inv_kin_reachable = rospy.ServiceProxy('/inverse_kinematics_reachability', InvKin)

        # Setup get_model_state service
        rospy.wait_for_service('/gazebo/get_model_state')
        self.model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        rospy.wait_for_service('/gazebo/get_link_state')
        self.link_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        # Setup tf2
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        file_name = str(Path.home()) + '/catkin_ws/src/path_planning/config/settings.yaml'
        with open(file_name) as yamlfile: # add right path
            self.APFyamlData = yaml.load(yamlfile, Loader=SafeLoader)
        #print(self.APFyamlData)

        # Setup gripper publisher
        self.gripper_publisher = rospy.Publisher("gripper_state", Bool, queue_size=10)

        self.point_pub = rospy.Publisher('/APF_Point', Point, queue_size=10)

    def move(self, pos:Pose, final_link_name:str, precise_orientation:bool):
        """ Move arm to specified position.

        INPUT: geometry_msgs Pose() - Orientation as quaternions

        Uses inverse_kinematics service.
        """

        rospy.wait_for_service('/inverse_kinematics')

        #rospy.loginfo("Path Planner - Service Helper - Calling ik for %s", self.robot_ns)

        inv_kin_request = InvKinRequest()

        # Initialise and fill ArmPos object
        arm_pos = ModelState()
        arm_pos.model_name = self.robot_ns
        arm_pos.reference_frame = final_link_name
        arm_pos.pose = pos
        #rospy.logerr("arm_pos: %s",arm_pos)
        inv_kin_request.state = arm_pos
        inv_kin_request.precise_orientation = precise_orientation

        # Call inverse_kinematics service and log ArmPos
        return self.inv_kin(inv_kin_request).success
    
    def moveGripper(self, state:bool):
        self.gripper_publisher.publish(state)
        
        #rospy.loginfo("Path Planner - Service Helper - Gripper set to state %i.", state)

    def getBlockPos(self, specific_model_name:str) -> Pose:
        """ Get block position relative to current robot arm
        INPUT: string specific_model_name
        OUTPUT: gazebo_msgs Pose() - Orientation in Euler angles not quaternions

        Uses gazebo/get_model_state service. REDUNDANT
        """
        # TODO: Replace with data from /blocks

        rospy.wait_for_service('/gazebo/get_model_state')

        # Extract Pose() object
        data = self.model_state_service(specific_model_name, "world").pose
        return data

    def frameConverter(self, target_frame:str, reference_frame:str, goal_pose:Pose) -> Pose:
        # Setup time stamped pose object
        start_pose = PoseStamped()
        start_pose.pose = goal_pose

        orientation_in_quaternion = [start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w]
        orientation_in_euler = tf_conversions.transformations.euler_from_quaternion(orientation_in_quaternion)

        #rospy.loginfo("Frame Converter - Start pose:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f", start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z, orientation_in_euler[0]*180/pi, orientation_in_euler[1]*180/pi, orientation_in_euler[2]*180/pi)

        start_pose.header.frame_id = reference_frame
        start_pose.header.stamp = rospy.get_rostime()

        # Convert from world frame to robot frame using tf2
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                new_pose = self.tfBuffer.transform(start_pose, target_frame)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("Error - Frame converter in Path Planner ServiceHelper.py failed. Retrying now.")
                rate.sleep()
                continue
        
        orientation_in_quaternion = [new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w]
        orientation_in_euler = tf_conversions.transformations.euler_from_quaternion(orientation_in_quaternion)
        
        #rospy.loginfo("Frame Converter - New pose:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f", new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z, orientation_in_euler[0]*180/pi, orientation_in_euler[1]*180/pi, orientation_in_euler[2]*180/pi)

        return new_pose.pose
    
    def getJointPos(self, ref_arm_name:str,target_arm_name:str,link:str) -> Pose:
        """ Get target cartesian joint coordinates from reference point
        INPUT: string ref_arm_name, string target_arm_name, string link
        OUTPUT: Pose 
        """
        joint_pos = Pose()
        #joint_header = Header()

        BaseID=ref_arm_name+'/base_link'
        linkID=target_arm_name+link
        #rospy.loginfo(BaseID,linkID)
        try:
            trans = self.tfBuffer.lookup_transform(linkID, BaseID, rospy.Time(0)) # get transform between base and link 
            joint_pos.position.x=trans.transform.translation.x #unit: meters
            joint_pos.position.y=trans.transform.translation.y
            joint_pos.position.z=trans.transform.translation.z

            joint_pos.orientation.x=trans.transform.rotation.x
            joint_pos.orientation.y=trans.transform.rotation.y
            joint_pos.orientation.z=trans.transform.rotation.z
            joint_pos.orientation.w=trans.transform.rotation.w
            #rospy.loginfo("Position of %s is %f, %f, %f in reference to %s",linkID, joint_pos.position.x,joint_pos.position.y,joint_pos.position.z,ref_arm_name)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException): 
            rospy.logerr("GetJointPos: Error Transformation not found between %s to %s",BaseID ,linkID)
        return joint_pos

    def getLinkPos(self, arm_name:str,link_name:str) -> Pose:
        """ Get Joint position relative to world?
        INPUT: string arm_name string link_name
        OUTPUT: gazebo_msgs Pose() - Orientation in Euler angles not quaternions
        Uses gazebo/get_link_state service.
        """
        # TODO: Replace with data from /blocks
        rospy.wait_for_service('/gazebo/get_link_state')

        try:
            # Extract Pose() object
            specific_link_name=arm_name+"::"+link_name
            data = self.link_state_service(specific_link_name, "world").link_state.pose
        except rospy.ServiceException:
            rospy.logfatal("Path Planner - Service Helper - getLinkPos failed.")
            data = None

        return data

    def EuclidianDistance(self,x,y,z,xgoal,ygoal,zgoal):
        d = ((x-xgoal)**2+(y-ygoal)**2+(z-zgoal)**2)**0.5 #absolute distance
        return d

    def EuclidianDistance2d(self,x,y,xgoal,ygoal):
        d = ((x-xgoal)**2+(y-ygoal)**2)**0.5
        return d

    def Link_Midpoints(self,xobj,yobj,zobj,Q): #interpolate points to reat
        Link_Granularity=self.APFyamlData["Link_Granularity"]
        no_links = len(xobj) -1
        newxobj = []
        newyobj = []
        newzobj = []
        newQ = []
        if no_links ==0:
            newxobj = xobj
            newyobj = yobj
            newzobj = zobj
            newQ = Q
        else:
            for i in range(no_links):
                vector = [xobj[i + 1] - xobj[i], yobj[i + 1] - yobj[i], zobj[i + 1] - zobj[i]]
                deltaQ = Q[i+1]-Q[i]
                for j in range(Link_Granularity):
                    newxobj.append(xobj[i] + vector[0]*j/Link_Granularity)
                    newyobj.append(yobj[i] + vector[1] * j / Link_Granularity)
                    newzobj.append(zobj[i] + vector[2] * j / Link_Granularity)
                    newQ.append(Q[i] + deltaQ*j/Link_Granularity)
        return newxobj,newyobj,newzobj,newQ

    def PotentialAttractionChange(self,x,y,z,xgoal,ygoal,zgoal,D): #attraction at a specific point (used to calculate)
        """ Calculates potential change from point to goal
        INPUT: current position and goal position XYs and distance where laws change. 
        OUTPUT: PotentialChange (a tuple of the change in potential along x and y axis (deltaX,deltaY))
        """
        Att_Change_SF = self.APFyamlData["Att_Change_SF"] #scaling factor
        d= self.EuclidianDistance(x,y,z,xgoal,ygoal,zgoal)
        if d <= D:
            PotentialChange = [Att_Change_SF*x-Att_Change_SF*xgoal,Att_Change_SF*y-Att_Change_SF*ygoal,Att_Change_SF*z-Att_Change_SF*zgoal]
        if d > D:
            PotentialChange = [(Att_Change_SF*x-Att_Change_SF*xgoal)/d,(Att_Change_SF*y-Att_Change_SF*ygoal)/d, (Att_Change_SF*z-Att_Change_SF*zgoal)/d]
        #rospy.loginfo('attraction change:',PotentialChange)
        return PotentialChange

    def PotentialAttraction(self,x,y,z,xgoal,ygoal,zgoal,D): #the attractive field as a whole (used to display)
        """ Calculates potential from point to goal
        INPUT: current position and goal position XYs and distance where laws change. 
        OUTPUT PotentialAtt (a single value for the Potential at those coordinates)
        """
        Att_SF = self.APFyamlData["Att_SF"] #scaling factor
        d = self.EuclidianDistance(x,y,z,xgoal,ygoal,zgoal)
        if d <= D:
            PotentialAtt = 0.5*Att_SF*(d**2)
        else:
            PotentialAtt = D*Att_SF*d-0.5*Att_SF*D
        return PotentialAtt

    def PotentialAttraction2d(self,x,y,xgoal,ygoal,D):
        Att_SF = self.APFyamlData["Att_SF"] #scaling factor
        d = self.EuclidianDistance2d(self,x,y,xgoal,ygoal)
        if d <= D:
            PotentialAtt = 0.5*Att_SF*(d**2)
        else:
            PotentialAtt = D*Att_SF*d-0.5*Att_SF*D
        return PotentialAtt
    
    def PotentialRepulsion(self,x,y,z,xobj,yobj,zobj,Q): #Repulsive field as a whole (used to display)
        """ 
        INPUT: current position and obstacle postition XYs. 
        OUTPUT: PotentialRep (a single value for Potential at those coordinates)
        """
        Rep_SF = self.APFyamlData["Rep_SF"]
        PotentialRep = 0
        for objNum in range(len(xobj)):
            d = self.EuclidianDistance(x,y,z,xobj[objNum],yobj[objNum],zobj[objNum])
            if d <= Q[objNum]:
                PotentialRepcurrent = Rep_SF*((1/d)-(1/Q[objNum]))
            else:
                PotentialRepcurrent = 0
            if PotentialRepcurrent > 100:
                PotentialRepcurrent = 100
            PotentialRep += PotentialRepcurrent
        return PotentialRep

    def PotentialRepulsion2d(self,x,y,xobj,yobj,Q):
        Rep_SF = self.APFyamlData["Rep_SF"]
        PotentialRep = 0
        for objNum in range(len(xobj)):
            d = self.EuclidianDistance2d(x,y,xobj[objNum],yobj[objNum])
            if d <= Q[objNum]:
                PotentialRepcurrent = Rep_SF * ((1/d)-(1/Q[objNum]))
            else:
                PotentialRepcurrent = 0
            if PotentialRepcurrent > 100:
                PotentialRepcurrent = 100
            PotentialRep += PotentialRepcurrent
        return PotentialRep

    def PotentialRepulsionChange(self,x,y,z,xobj,yobj,zobj,xgoal,ygoal,zgoal,Q): #repulsion at a specific point (used to calculate)
        """
        INPUT: current position and obstacle position XYs. 
        OUTPUT: is repulsion at a specific point
        """
        Rep_Change_SF = self.APFyamlData["Rep_Change_SF"]
        allvectorsx = 0
        allvectorsy = 0
        allvectorsz = 0
        repulsionangle = 0
        repulsionvect = 0,0
        zrep = 0
        for objNum in range(len(xobj)):
            #generate the vectors and angles
            homevect = [xgoal-x,ygoal-y,zgoal-z]
            objvect = (xobj[objNum]-x,yobj[objNum]-y,zobj[objNum]-z) # angles are ebcoming negative which causes wrogn ddirection
            anglegoal = math.atan2(homevect[1] ,homevect[0])
            angleobj = math.atan2(objvect[1] ,objvect[0])
            #IS THIS THE RIGHT WAY ROUND? - NEEDS TO BE OBJ - GOAL
            angle =  angleobj-anglegoal   
            #rospy.loginfo("ANGLE - %.2f",angle)
            zheight = z-zobj[objNum]
            d = self.EuclidianDistance2d(x ,y ,xobj[objNum] ,yobj[objNum])
            D = self.EuclidianDistance(x ,y ,z ,xobj[objNum] ,yobj[objNum] ,zobj[objNum])
            zangle = math.atan2(zheight,d)
            # deciding the direction of the tangent
            ##rospy.logwarn("Distance; %.2f",d)
            if d == 0:
                d = 0.0001
            scalings = (1/d**2)*(1/Q[objNum]-1/d)
            if scalings == 0:
                0.00001
            if angle < 0:
                #rospy.logwarn("GO LEFT")
                repulsionangle =anglegoal+100
                repulsionvect =-Rep_Change_SF*scalings*(objvect[0]*math.cos(100)-objvect[1]*math.sin(100)),-Rep_Change_SF*scalings*(objvect[0]*math.sin(100)+objvect[1]*math.cos(100))
            if angle > 0 or angle == 0:
                #rospy.logwarn("GO RIGHT")
                repulsionangle = anglegoal-100
                repulsionvect = -Rep_Change_SF*scalings*(objvect[0]*math.cos(-100)-objvect[1]*math.sin(-100)),-Rep_Change_SF*scalings*(objvect[0]*math.sin(-100)+objvect[1]*math.cos(-100))
            if zheight >= 0:
                #rospy.loginfo("GO UP")
                zrepangle = zangle - 100
                zrep = -Rep_Change_SF*(1/D**2)*(1/Q[objNum]-1/D)*zrep*math.cos(100)
            if zheight < 0:
                #rospy.loginfo("GO DOWN")
                zrepangle = zangle+100
                zrep = -Rep_Change_SF*(1/D**2)*(1/Q[objNum]-1/D)*zrep*math.cos(-100)
            #deciding whether the obstacle is in range
            #if D<Q[objNum]:
                #rospy.logwarn("in influence")
            #rospy.loginfo("repchange: %.2f ")
            #repulsionvect = Rep_Change_SF*math.cos(math.radians(repulsionangle)),Rep_Change_SF*math.sin(math.radians(repulsionangle))
            #repulsionvect = list(repulsionvect)
            #zrep = Rep_Change_SF*math.cos(math.radians(zangle))*math.sin(math.radians(zrepangle))
            if D > Q[objNum] or abs(angle) >90:
                repulsionvect = 0,0
                zrep = 0
            else:
                pass
                #rospy.loginfo("zinfo: %.2f,%.2f,%.2f",zheight,zangle,zrepangle)
            #for x in range(len(repulsionvect)):
            #   if repulsionvect[x] > 3:
            #      repulsionvect[x]= 3
            allvectorsx += repulsionvect[0]
            allvectorsy += repulsionvect[1]
            allvectorsz += zrep
        return allvectorsx,allvectorsy,allvectorsz

    def is_block_reachable_APF(self, X, Y, Z):
        rospy.wait_for_service('/inverse_kinematics_reachability')
        inv_kin_is_reachable = rospy.ServiceProxy('/inverse_kinematics_reachability', InvKin)
        
        inv_kin_request = InvKinRequest()
        model_state = ModelState()

        model_state.pose.position.x = X/100
        model_state.pose.position.y = Y/100
        model_state.pose.position.z = Z/100

        block_orientation_quaternion = [model_state.pose.orientation.x, model_state.pose.orientation.y, model_state.pose.orientation.z, model_state.pose.orientation.w]
        block_orientation_euler = tf_conversions.transformations.euler_from_quaternion(block_orientation_quaternion)

        orientation_euler = [0, math.pi, block_orientation_euler[2]]
        orientation_quaternion = tf_conversions.transformations.quaternion_from_euler(orientation_euler[0], orientation_euler[1], orientation_euler[2])
        
        model_state.pose.orientation.x = orientation_quaternion[0]
        model_state.pose.orientation.y = orientation_quaternion[1]
        model_state.pose.orientation.z = orientation_quaternion[2]
        model_state.pose.orientation.w = orientation_quaternion[3]

        inv_kin_request.state = model_state
        inv_kin_request.precise_orientation = True

        if inv_kin_is_reachable(inv_kin_request).success:
            return True
            
        return False
        
    def PathPlanner(self,x,y,z,xgoal,ygoal,zgoal,xobj,yobj,zobj,Q,D,tempxobj,tempyobj,tempzobj,tempQ,precise_angle_flag): #you are currently trying to add this in, this is the path from a poiint using position and force ads velocity
        """   ## make temp inside of move)()
        returned as an array of points
        INPUT: start position and goal position XYs, xobj and yobj (array of obstacle x/y points)   
        OUTPUT: PathPoints (an array of the via points ((x1,y1),(x2,y2),(x3,y3)....))
        """
        if precise_angle_flag:
            Step_Size = self.APFyamlData["Step_Size_Precise"]
        else:
            Step_Size = self.APFyamlData["Step_Size"]
            
        Final_Att= self.APFyamlData["Final_Att"]
        Final_Distance = self.APFyamlData["Final_Distance"]
        PathComplete = 0 #This turns to 1 and ends the function once end effector has reached target position (minimum of potential)
        PathPointsx = [x] #First X and Y points
        PathPointsy = [y] #These are in different arrays cos tuples suck. The 'zip' function at the end turns them into a tuple
        PathPointsz = [z]
        i = 0
        d = self.EuclidianDistance(x,y,z,xgoal,ygoal,zgoal)
        diffrep = self.PotentialRepulsionChange(PathPointsx[i],PathPointsy[i],PathPointsz[i],xobj,yobj,zobj,xgoal,ygoal,zgoal,Q)
        #diffreptemp = self.PotentialRepulsionChange(PathPointsx[i],PathPointsy[i],PathPointsz[i],tempxobj,tempyobj,tempzobj,xgoal,ygoal,zgoal,tempQ)
        diffatt = self.PotentialAttractionChange(PathPointsx[i],PathPointsy[i],PathPointsz[i],xgoal,ygoal,zgoal,D)
        if any(diffrep) != 0:
            difx = diffrep[0] + 0.25*diffatt[0]
            dify = diffrep[1] + 0.25*diffatt[1]
            difz = diffrep[2] + 0.25*diffatt[2]
                #rospy.logwarn("Potential Fields - Repulsion strength: %.2f,%.2f,%.2f dist: %.2f",-diffrep[0],-diffrep[1],-diffrep[2],d)
            #rospy.logwarn("Potential Fields - Repulsion strength: %.2f,%.2f,%.2f dist: %.2f",-diffrep[0],-diffrep[1],-diffrep[2],d)

        else:
            difx = diffatt[0]
            dify = diffatt[1]
            difz = diffatt[2]
            #rospy.loginfo("Potential Fields - Attraction strength: %.2f,%.2f,%.2f dist: %.2f",-diffatt[0],-diffatt[1],-diffatt[2],d)
            #rospy.logwarn("Potential Fields - Attraction strength: %.2f,%.2f,%.2f dist: %.2f",-difx,-dify,-difz,d)
            #rospy.loginfo("Temporary Objects: %.2f",len(tempxobj))
        
        if abs(difx) <Final_Att and abs(dify) <Final_Att and abs(difz) <Final_Att and d < Final_Distance:#
            PathComplete = 1
        else:
            #rospy.loginfo('Iteration: ',i,'x,y: ',PathPointsx,PathPointsy)
            nextx = PathPointsx[i] - Step_Size*difx
            nexty = PathPointsy[i] - Step_Size*dify
            nextz = PathPointsz[i] - Step_Size*difz
            x = nextx
            y = nexty
            z = nextz
            if z < -0.2:
                z = -0.2
            #if self.is_block_reachable_APF(x,y,z) == False:
                #tempxobj.append(x)
                #tempyobj.append(y)
                #tempzobj.append(z) 
                #rospy.loginfo("APF Planner - Point is not reachable by %s, added tempobj at %.2f %.2f %.2f", self.robot_ns, x,y,z)
                #problem - if reachablility fucks up and says it can't reach, then it'll place an object on top of the block :(
                #may need to check if it can't reach AND it's out of bounds, the IK checking is not foolproof
                #another problem - reachability seems to fail when very close to the block on various block positions
                #PathPointsx.append(PathPointsx[i])
                #PathPointsy.append(PathPointsy[i])
                #PathPointsz.append(PathPointsz[i])
                #objdistance = 1.6*self.EuclidianDistance(PathPointsx[i],PathPointsy[i],PathPointsz[i],x,y,z) 
                #added Q scaling factor so Q is greater than distance to next point
                #tempQ.append(objdistance)
                #else:
                #PathPointsx.append(x)
                #PathPointsy.append(y)
                #PathPointsz.append(z)
                #rospy.loginfo('Path Points: %.2f %.2f %.2f',x,y,z)
                #i += 1
            #rospy.loginfo(PathPointsx[i],PathPointsy[i])
        #PathPoints = list(zip(PathPointsx,PathPointsy))
        self.publish_path_points(x,y,z)
        return x, y, z

    def publish_path_points(self,x,y,z):
        """  publish path points
        INPUT: xyz points   
        OUTPUT: publishes point to be used by gui
        """
        point = Point()
        point.x = x
        point.y = y
        point.z = z

        self.point_pub.publish(point)
        return

    def is_obsarm_in_zone(self,robot_namespaces,xgoal,ygoal):
        """  tell arm if other arms are in dropoff zone
        INPUT: xyz points   
        OUTPUT: publishes point to be used by gui
        """
        obs_in_zone_flag = 0
        forcefield_dist = 0.20
        pos_own_world = self.getLinkPos(self.robot_ns,"link6") #own arm joint positions relative to world
        d_own_to_goal = self.EuclidianDistance2d(pos_own_world.position.x, pos_own_world.position.y, xgoal, ygoal)

        for obstacle_arm_ns in robot_namespaces:
            pos_obs_world = self.getLinkPos(obstacle_arm_ns,"link6") #obstacle arm joint positions relative to world
            d_obs_to_goal = self.EuclidianDistance2d(pos_obs_world.position.x, pos_obs_world.position.y, xgoal, ygoal) #distance of other arm to zone

            if d_obs_to_goal <= forcefield_dist: #if other arm is in forcefield
                if d_own_to_goal <= forcefield_dist: #if own arm is also in forcefield
                    rospy.logwarn("Path Planner - Both arms in forcefield area")
                    if d_own_to_goal >= d_obs_to_goal: #if other arm nearer
                        obs_in_zone_flag = 1

                else: #own arm not in forcefield
                    obs_in_zone_flag = 1

        return obs_in_zone_flag
