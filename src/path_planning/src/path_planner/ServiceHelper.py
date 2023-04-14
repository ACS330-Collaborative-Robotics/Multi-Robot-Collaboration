# Name: ServiceHelper Class Definition
# Authors: Conor Nichols (cjnichols1@sheffield.ac.uk) Joseph Fields (jfields1@sheffield.ac.uk)

import rospy
import tf2_ros
import tf

from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetLinkState
from geometry_msgs.msg import Pose
from tf2_geometry_msgs import PoseStamped
from inv_kinematics.srv import InvKin
from inv_kinematics.srv import InvKinRequest
from std_msgs.msg import Header


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
        self.target_block=target_block
        # Setup inverse_kinematics service
        rospy.wait_for_service('inverse_kinematics')
        self.inv_kin = rospy.ServiceProxy('inverse_kinematics', InvKin)

        # Setup get_model_state service
        rospy.wait_for_service('gazebo/get_model_state')
        self.model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

        rospy.wait_for_service('gazebo/get_link_state')
        self.link_state_service = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)

        # Setup tf2
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        with open('/home/stevencraig147/catkin_ws/src/path_planning/config/settings.yaml') as yamlfile: # add rright path
            self.APFyamlData = yaml.load(yamlfile, Loader=SafeLoader)
        print(self.APFyamlData)

    def move(self, pos:Pose, final_link_name:str, precise_orientation:bool):
        """ Move arm to specified position.

        INPUT: geometry_msgs Pose() - Orientation as quaternions

        Uses inverse_kinematics service.
        """
        rospy.wait_for_service('inverse_kinematics')

        rospy.loginfo("Path Planner - Service Helper - Calling ik for %s for %s", self.robot_ns,self.target_block)

        inv_kin_request = InvKinRequest()

        # Initialise and fill ArmPos object
        arm_pos = ModelState()
        arm_pos.model_name = self.robot_ns
        arm_pos.reference_frame = final_link_name

        arm_pos.pose = pos

        inv_kin_request.state = arm_pos
        inv_kin_request.precise_orientation = precise_orientation

        # Call inverse_kinematics service and log ArmPos
        return self.inv_kin(inv_kin_request).success
    
    def getBlockPos(self, specific_model_name:str) -> Pose:
        """ Get block position relative to current robot arm
        INPUT: string specific_model_name
        OUTPUT: gazebo_msgs Pose() - Orientation in Euler angles not quaternions

        Uses gazebo/get_model_state service.
        """
        # TODO: Replace with data from /blocks
        rospy.wait_for_service('gazebo/get_model_state')
        # Extract Pose() object
        data = self.model_state_service(specific_model_name, "world").pose
        return data

    def frameConverter(self, target_frame:str, reference_frame:str, goal_pose:Pose) -> Pose:
        # Setup time stamped pose object
        start_pose = PoseStamped()
        start_pose.pose = goal_pose

        #rospy.loginfo("Frame Converter - Start pose:\t%.2f\t%.2f\t%.2f", start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z)

        start_pose.header.frame_id = reference_frame
        start_pose.header.stamp = rospy.get_rostime()

        # Convert from world frame to robot frame using tf2
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                new_pose = self.tfBuffer.transform(start_pose, target_frame)
                #rospy.loginfo("Frame Converter - New pose:%.2f,%.2f,%.2f", new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Error - Frame converter in Path Planner ServiceHelper.py failed. Retrying now.")
                rate.sleep()
                continue
        
        #rospy.loginfo("Frame Converter - New pose:\t%.2f\t%.2f\t%.2f", new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z)

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
            rospy.logerr("GetJointPos: Error Transformation not found between %s to %s",BaseID,linkID)
        return joint_pos

    def getLinkPos(self, arm_name:str,link_name:str) -> Pose:
        """ Get Joint position relative to current robot arm
        INPUT: string arm_name string link_name
        OUTPUT: gazebo_msgs Pose() - Orientation in Euler angles not quaternions
        Uses gazebo/get_link_state service.
        """
        # TODO: Replace with data from /blocks
        rospy.wait_for_service('gazebo/get_link_state')
        # Extract Pose() object
        specific_link_name=arm_name+"::"+link_name
        data = self.link_state_service(specific_link_name, "world").link_state.pose
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
            PotentialChange = [(Att_Change_SF*x-Att_Change_SF*xgoal)/d,(Att_Change_SF*y-Att_Change_SF*ygoal)/d,(Att_Change_SF*z-Att_Change_SF*zgoal)/d]
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
            PotentialAtt = D*Att_SF*d - 0.5*Att_SF*D
        return PotentialAtt

    def PotentialAttraction2d(self,x,y,xgoal,ygoal,D):
        Att_SF = self.APFyamlData["Att_SF"] #scaling factor
        d = self.EuclidianDistance2d(self,x,y,xgoal,ygoal)
        if d <= D:
            PotentialAtt = 0.5*Att_SF*(d**2)
        else:
            PotentialAtt = D*Att_SF*d - 0.5*Att_SF*D
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
                PotentialRepcurrent = Rep_SF*((1/d)-(1/Q[objNum]))
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
        for objNum in range(len(xobj)):
            #generate the vectors and angles
            homevect = [xgoal-x,ygoal-y,zgoal-z]
            objvect = (xobj[objNum]-x,yobj[objNum]-y,zobj[objNum]-z)
            anglegoal = math.atan2(homevect[1],homevect[0])
            angleobj = math.atan2(objvect[1],objvect[0])
            angle = angleobj-anglegoal
            zheight = z-zobj[objNum]
            d = self.EuclidianDistance2d(x,y,xobj[objNum],yobj[objNum])
            D = self.EuclidianDistance(x,y,z,xobj[objNum],yobj[objNum],zobj[objNum])
            zangle = math.atan2(zheight,d)
            # deciding the direction of the tangent
            if angle > 0 or angle == 0:
                repulsionangle = anglegoal + 100
            if angle < 0:
                repulsionangle = anglegoal - 100

            if zheight >= 0:
                zrepangle = zangle - 100
            if zheight < 0:
                zrepangle = zangle + 100

            #deciding whether the obstacle is in range
            #if D<Q[objNum]:
                #rospy.loginfo("in influence")
            repulsionvect = Rep_Change_SF*math.cos(math.radians(angle))*math.cos(math.radians(repulsionangle)),Rep_Change_SF*math.cos(math.radians(angle))*math.sin(math.radians(repulsionangle))
            repulsionvect = list(repulsionvect)
            zrep = Rep_Change_SF*math.cos(math.radians(zangle))*math.sin(math.radians(zrepangle))
            if D > Q[objNum]:
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

    def is_block_reachable_APF(self,X,Y,Z, robot_namespaces):
        rospy.wait_for_service('inverse_kinematics_reachability')
        inv_kin_is_reachable = rospy.ServiceProxy('inverse_kinematics_reachability', InvKin)
    
         # Create Initial Pose object
        initial_pose = Pose()
        initial_pose.position.x = X
        initial_pose.position.y = Y
        initial_pose.position.z = Z
        quat = tf.transformations.quaternion_from_euler(initial_pose.position.x, initial_pose.position.y, initial_pose.position.z)
        initial_pose.orientation.x = quat[0]
        initial_pose.orientation.y = quat[1]
        initial_pose.orientation.z = quat[2]
        initial_pose.orientation.w = quat[3]

        inv_kin_request = InvKinRequest()
        #model_state = ModelState()

        # for robot_name in robot_namespaces:
        #    model_state.pose = specific_block_pose(block_name, robot_name)

        orientation_in_euler = [0,90*math.pi/180,0]
        orientation = tf.transformations.quaternion_from_euler(orientation_in_euler[0], orientation_in_euler[1], orientation_in_euler[2])
        
        initial_pose.orientation.x = orientation[0]
        initial_pose.orientation.y = orientation[1]
        initial_pose.orientation.z = orientation[2]
        initial_pose.orientation.w = orientation[3]
        initial_pose.position.z += 0.15

        inv_kin_request.state.model_name=self.robot_ns
        
        inv_kin_request.state.pose = initial_pose
        inv_kin_request.precise_orientation = False

        if inv_kin_is_reachable(inv_kin_request).success:
            rospy.loginfo("Assignment Selection - Adding %s as it is reachable by %s", self.target_block, self.robot_ns)
            return True
        else:
            return False
        
    def PathPlanner(self,x,y,z,xgoal,ygoal,zgoal,xobj,yobj,zobj,Q,D): #you are currently trying to add this in, this is the path from a poiint using position and force ads velocity
        """
        returned as an array of points
        INPUT: start position and goal position XYs, xobj and yobj (array of obstacle x/y points)   
        OUTPUT: PathPoints (an array of the via points ((x1,y1),(x2,y2),(x3,y3)....))
        """
        tempxobj = []
        tempyobj = []
        tempzobj = []
        tempQ = []
        Step_Size= self.APFyamlData["Step_Size"]
        Final_Att= self.APFyamlData["Final_Att"]
        Final_Distance= self.APFyamlData["Final_Distance"]
        PathComplete = 0 #This turns to 1 and ends the function once end effector has reached target position (minimum of pootential)
        PathPointsx = [x] #First X and Y points
        PathPointsy = [y] #These are in different arrays cos tuples suck. The 'zip' function at the end turns them into a tuple
        PathPointsz = [z]
        i = 0
        while PathComplete == 0 and not rospy.is_shutdown():
            d = self.EuclidianDistance(x,y,z,xgoal,ygoal,zgoal)
            diffrep = self.PotentialRepulsionChange(PathPointsx[i],PathPointsy[i],PathPointsz[i],xobj,yobj,zobj,xgoal,ygoal,zgoal,Q)
            diffreptemp = self.PotentialRepulsionChange(PathPointsx[i],PathPointsy[i],PathPointsz[i],tempxobj,tempyobj,tempzobj,xgoal,ygoal,zgoal,tempQ)
            diffatt = self.PotentialAttractionChange(PathPointsx[i],PathPointsy[i],PathPointsz[i],xgoal,ygoal,zgoal,D)
            if any(diffrep) != 0:
                difx = diffrep[0] + diffreptemp[0] + 0.25*diffatt[0]
                dify = diffrep[1] + diffreptemp[1] + 0.25*diffatt[1]
                difz = diffrep[2] + diffreptemp[2] + 0.25*diffatt[2]
                #rospy.loginfo("Potential Fields - Repulsion strength: %.2f,%.2f,%.2f dist: %.2f",-difx,-dify,-difz,d)
            else:
                difx = diffatt[0]
                dify = diffatt[1]
                difz = diffatt[2]
                #rospy.loginfo("Potential Fields - Attraction strength: %.2f,%.2f,%.2f dist: %.2f",-difx,-dify,-difz,d)

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
                if z < 0.05:
                    z = 0.05
                #if self.is_block_reachable_APF(x,y,z,'mover6_a') == False:
                 #   tempxobj.append(x)
                  #  tempyobj.append(y)
                   # tempzobj.append(z)
                    #PathPointsx.append(PathPointsx[i])
                    #PathPointsy.append(PathPointsy[i])
                    #PathPointsz.append(PathPointsz[i])
                    #objdistance = self.EuclidianDistance(PathPointsx[i],PathPointsy[i],PathPointsz[i],x,y,z)
                    #tempQ.append(objdistance)
                #else:
                PathPointsx.append(x)
                PathPointsy.append(y)
                PathPointsz.append(z)
                rospy.loginfo('Path Points %.2f %.2f  %.2f',PathPointsx[i],PathPointsy[i],PathPointsz[i])
                i += 1
            #rospy.loginfo(PathPointsx[i],PathPointsy[i])
        #PathPoints = list(zip(PathPointsx,PathPointsy))
        return PathPointsx,PathPointsy,PathPointsz

    def Space_Generation(self,startx,starty,startz,xgoal,ygoal,zgoal,xobj,yobj,zobj,Q,D): #### needs to ad objx and objy
        x = np.linspace(-50, 50, 100)  # Creating X and Y axis
        y = np.linspace(-50, 50, 100)
        X, Y = np.meshgrid(x, y)  # Creates 2 arrays with respective x any y coordination for each point
        PotentialEnergy = np.ndarray(shape=(len(x), len(y)))  # this acts as the z axis on graphs. Works better for visualisation
        for i in range(len(X)):  # gets Z values for the X Y positions
            for j in range(len(Y)):
                PotentialEnergy[i, j] = self.PotentialAttraction2d(X[i,j],Y[i,j],xgoal,ygoal,D)+ self.PotentialRepulsion2d(X[i,j],Y[i,j],xobj,yobj,Q)
                         # PotentialAttraction(X[i,j],Y[i,j],xgoal,ygoal,D) +PotentialRepulsion(X[i, j], Y[i, j], objx, objy,
        PathTaken = self.PathPlanner(startx, starty,startz, xgoal, ygoal,zgoal, xobj, yobj,zobj,Q, D)  ## you are here ^^^
        EnergyPathTaken = []
        xline = PathTaken[0]
        yline = PathTaken[1]
        for i in range(len(PathTaken[0])):

            TotalPotential = self.PotentialAttraction2d(xline[i], yline[i], xgoal, ygoal, D) + self.PotentialRepulsion2d(xline[i], yline[i], xobj, yobj, Q)
            EnergyPathTaken.append(TotalPotential)
        rospy.loginfo('Space Generation Complete')
        return X,Y,xline, yline, PotentialEnergy, EnergyPathTaken, PathTaken

    def plotAPF(self,X,Y, xline, yline, PotentialEnergy,EnergyPathTaken):
    # Making 3d Plot
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot_surface(X, Y, PotentialEnergy)
        ax.plot(xline, yline, EnergyPathTaken, color='red', linewidth=4.5)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        plt.show()

    def plotPath(self,PathTaken):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        xpoints =[]
        ypoints = []
        zpoints = []
        for point in PathTaken:
            xpoints.append(point[0])
            ypoints.append(point[1])
            zpoints.append(point[2])
        ax.plot(xpoints,ypoints,zpoints)
        plt.show()
        rospy.loginfo('PlotPath Complete')

    