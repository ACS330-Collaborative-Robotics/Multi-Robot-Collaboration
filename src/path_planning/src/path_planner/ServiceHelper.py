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
from std_msgs.msg import Header

#from APF dependancies
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

import math
from math import *

class ServiceHelper:
    def __init__(self, robot_ns):
        self.robot_ns = robot_ns

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

    def move(self, pos:Pose):
        """ Move arm to specified position.

        INPUT: geometry_msgs Pose() - Orientation as quaternions

        Uses inverse_kinematics service.
        """
        rospy.wait_for_service('inverse_kinematics')

        print("Path Planner - Service Helper - Calling ik for ", self.robot_ns)

        # Initialise and fill ArmPos object
        arm_pos = ModelState()
        arm_pos.model_name = self.robot_ns
        arm_pos.pose = pos
        # Call inverse_kinematics service and log ArmPos
        return self.inv_kin(arm_pos)

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

        print("Frame Converter - Start pose:", start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z)
        #print(start_pose)

        start_pose.header.frame_id = reference_frame
        start_pose.header.stamp = rospy.get_rostime()

        # Convert from world frame to robot frame using tf2
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                new_pose = self.tfBuffer.transform(start_pose, target_frame)
                print("Frame Converter - New pose:", new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Error - Frame converter in Path Planner ServiceHelper.py failed. Retrying now.")
                rate.sleep()
                continue
        #print(new_pose)
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
        #print(BaseID,linkID)
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
        no_links = len(xobj) -1
        newxobj = []
        newyobj = []
        newzobj = []
        newQ = []
        for i in range(no_links):
            vector = [xobj[i + 1] - xobj[i], yobj[i + 1] - yobj[i], zobj[i + 1] - zobj[i]]
            deltaQ = Q[i+1]-Q[i]
            for j in range(10):
                newxobj.append(xobj[i] + vector[0]*j/10)
                newyobj.append(yobj[i] + vector[1] * j / 10)
                newzobj.append(zobj[i] + vector[2] * j / 10)
                newQ.append(Q[i] + deltaQ*j/10)

        print('Midpoints Complete')
        return newxobj,newyobj,newzobj,newQ

    def NoiseAddition(self,PathPointsx,PathPointsy,xgoal,ygoal,xobj,yobj): #what does this do?
        ObjAngle = math.atan2(ygoal-PathPointsy,xobj-xgoal)
        ObjAngle = math.degrees(ObjAngle)
        fig = plt.figure()
        ax = plt.axes()
        ax.plot([xgoal,PathPointsx,xobj],[ygoal,PathPointsy,yobj])
        ax.set_xlabel(ObjAngle)
        plt.show()
        return ObjAngle


    def PotentialAttractionChange(self,x,y,z,xgoal,ygoal,zgoal,D): #attraction at a specific point (used to calculate)
        """ Calculates potential change from point to goal
        INPUT: current position and goal position XYs and distance where laws change. 
        OUTPUT: PotentialChange (a tuple of the change in potential along x and y axis (deltaX,deltaY))
        """
        SF = 0.9 #scaling factor
        d= self.EuclidianDistance(x,y,z,xgoal,ygoal,zgoal)
        if d <= D:
            PotentialChange = [SF*x-SF*xgoal,SF*y-SF*ygoal,SF*z-SF*zgoal]
        if d > D:
            PotentialChange = [(SF*x-SF*xgoal)/d,(SF*y-SF*ygoal)/d,(SF*z-SF*zgoal)/d]
        #print('attraction change:',PotentialChange)
        return PotentialChange

    def PotentialAttraction(self,x,y,z,xgoal,ygoal,zgoal,D): #the attractive field as a whole (used to display)
        """ Calculates potential from point to goal
        INPUT: current position and goal position XYs and distance where laws change. 
        OUTPUT PotentialAtt (a single value for the Potential at those coordinates)
        """
        SF = 0.2 #scaling factor
        d = self.EuclidianDistance(x,y,z,xgoal,ygoal,zgoal)
        if d <= D:
            PotentialAtt = 0.5*SF*(d**2)
        else:
            PotentialAtt = D*SF*d - 0.5*SF*D
        return PotentialAtt

    def PotentialAttraction2d(self,x,y,xgoal,ygoal,D):
        SF = 0.2 #scaling factor
        d = EuclidianDistance2d(self,x,y,xgoal,ygoal)
        if d <= D:
            PotentialAtt = 0.5*SF*(d**2)
        else:
            PotentialAtt = D*SF*d - 0.5*SF*D
        return PotentialAtt
    
    def PotentialRepulsion(self,x,y,z,xobj,yobj,zobj,Q): #Repulsive field as a whole (used to display)
        """ 
        INPUT: current position and obstacle postition XYs. 
        OUTPUT: PotentialRep (a single value for Potential at those coordinates)
        """
        SF = 100
        PotentialRep = 0
        for objNum in range(len(xobj)):
            d = self.EuclidianDistance(x,y,z,xobj[objNum],yobj[objNum],zobj[objNum])
            if d <= Q[objNum]:
                PotentialRepcurrent = SF*((1/d)-(1/Q[objNum]))
            else:
                PotentialRepcurrent = 0
            if PotentialRepcurrent > 100:
                PotentialRepcurrent = 100
            PotentialRep += PotentialRepcurrent
        return PotentialRep

    def PotentialRepulsion2d(self,x,y,xobj,yobj,Q):
        SF = 100
        PotentialRep = 0
        for objNum in range(len(xobj)):
            d = EuclidianDistance2d(x,y,xobj[objNum],yobj[objNum])
            if d <= Q[objNum]:
                PotentialRepcurrent = SF*((1/d)-(1/Q[objNum]))
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
        allvectorsx = 0
        allvectorsy = 0
        allvectorsz = 0
        repulsionangle = 0
        for objNum in range(len(xobj)):
            homevect = [xgoal-x,ygoal-y,zgoal-z]
            objvect = (xobj[objNum]-x,yobj[objNum]-y,zobj[objNum]-z)
            anglegoal = math.atan2(homevect[1],homevect[0])
            angleobj = math.atan2(objvect[1],objvect[0])
            angle = angleobj-anglegoal
            zheight = objvect[2]-homevect[2]
            if angle > 0 or angle == 0:
                repulsionangle = anglegoal - 90
            if angle < 0:
                repulsionangle = anglegoal + 90
            d = self.EuclidianDistance(x,y,z,xobj[objNum],yobj[objNum],zobj[objNum])
            SF = 5*(d-Q[objNum])
            repulsionvect = SF*math.cos(angle)*math.cos(repulsionangle),SF*math.cos(angle)*math.sin(repulsionangle)
            if d > Q[objNum]:
                repulsionvect = 0,0
                zrep = 0
            else:
                zrep = zheight*1/(d-Q[objNum])
            allvectorsx += repulsionvect[0]
            allvectorsy += repulsionvect[1]
            allvectorsz += zrep
        return allvectorsx,allvectorsy,allvectorsz

    
    def PathPlanner(self,x,y,z,xgoal,ygoal,zgoal,xobj,yobj,zobj,Q,D): #you are currently trying to add this in, this is the path from a poiint using position and force ads velocity
        """
        returned as an array of points
        INPUT: start position and goal position XYs, xobj and yobj (array of obstacle x/y points)   
        OUTPUT: PathPoints (an array of the via points ((x1,y1),(x2,y2),(x3,y3)....))
        """
        PathComplete = 0 #This turns to 1 and ends the function once end effector has reached target position (minimum of pootential)
        PathPointsx = [x] #First X and Y points
        PathPointsy = [y] 
        PathPointsz = [z]
        #print("INITIAL PATH POINT: ",PathPointsx,PathPointsy)
        i = 0
        while PathComplete == 0:
            
            diffatt = self.PotentialAttractionChange(PathPointsx[i],PathPointsy[i],PathPointsz[i],xgoal,ygoal,zgoal,D)
            diffrep = self.PotentialRepulsionChange(PathPointsx[i],PathPointsy[i],PathPointsz[i],xobj,yobj,zobj,xgoal,ygoal,zgoal,Q)
            difx = diffatt[0] + diffrep[0]
            dify = diffatt[1] + diffrep[1]
            difz = diffatt[2] + diffrep[2]
            d = self.EuclidianDistance(x,y,z,xgoal,ygoal,zgoal)
            if abs(difx) <0.1 and abs(dify) <0.1 and abs(difz) <0.1 and d < 0.5:
                PathComplete = 1
            if abs(difx) < 0.05 and abs(dify) < 0.05 and d > 0.5:
                rospy.loginfo("LOCAL MINIMA")
                pass
                #add get out of minima here
            else:
                print('Iteration: ',i,'x,y,z: ',PathPointsx[i],PathPointsy[i],PathPointsz[i],' Dist:' ,d)
                nextx = PathPointsx[i] - 1.5*difx
                nexty = PathPointsy[i] - 1.5*dify
                nextz = PathPointsz[i] - 1.5*difz
                x = nextx
                y = nexty
                z = nextz
                PathPointsx.append(x)
                PathPointsy.append(y)
                PathPointsz.append(z)
            i += 1
        print('Path Complete')
        print(len(PathPointsx))
        return PathPointsx,PathPointsy,PathPointsz

 
    def Space_Generation(self,startx,starty,startz,xgoal,ygoal,zgoal,xobj,yobj,zobj,Q,D): #### needs to ad objx and objy
        x = np.linspace(-50, 50, 100)  # Creating X and Y axis
        y = np.linspace(-50, 50, 100)
        X, Y = np.meshgrid(x, y)  # Creates 2 arrays with respective x any y coordination for each point
        PotentialEnergy = np.ndarray(shape=(len(x), len(y)))  # this acts as the z axis on graphs. Works better for visualisation
        for i in range(len(X)):  # gets Z values for the X Y positions
            for j in range(len(Y)):
                PotentialEnergy[i, j] = PotentialAttraction2d(X[i,j],Y[i,j],xgoal,ygoal,D)+ PotentialRepulsion2d(X[i,j],Y[i,j],xobj,yobj,Q)
                         # PotentialAttraction(X[i,j],Y[i,j],xgoal,ygoal,D) +PotentialRepulsion(X[i, j], Y[i, j], objx, objy,
        PathTaken = PathPlanner(startx, starty,startz, xgoal, ygoal,zgoal, xobj, yobj,zobj,Q, D)  ## you are here ^^^
        EnergyPathTaken = []
        xline = PathTaken[0]
        yline = PathTaken[1]
        for i in range(len(PathTaken[0])):

            TotalPotential = PotentialAttraction2d(xline[i], yline[i], xgoal, ygoal, D) + PotentialRepulsion2d(xline[i], yline[i], xobj, yobj, Q)
            EnergyPathTaken.append(TotalPotential)
        print(EnergyPathTaken)
        print('Space Generation Complete')
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
        print("Successfuly run")

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
        print('PlotPath Complete')