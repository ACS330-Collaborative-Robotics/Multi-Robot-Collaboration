# Name: ServiceHelper Class Definition
# Authors: Conor Nichols (cjnichols1@sheffield.ac.uk) Joseph Fields (jfields1@sheffield.ac.uk)

import rospy
import tf2_ros
import tf

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
from tf2_geometry_msgs import PoseStamped
from inv_kinematics.srv import InvKin
from std_msgs.msg import Header

#from APF dependancies
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import math
from math import atan2, asin

class ServiceHelper:
    def __init__(self, robot_ns):
        self.robot_ns = robot_ns

        # Setup inverse_kinematics service
        rospy.wait_for_service('inverse_kinematics')
        self.inv_kin = rospy.ServiceProxy('inverse_kinematics', InvKin)

        # Setup get_model_state service
        rospy.wait_for_service('gazebo/get_model_state')
        self.model_state_service = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

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
        self.inv_kin(arm_pos)

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
                new_pose = self.tfBuffer.transform(start_pose, target_frame+"_base")
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Error - Frame converter in Path Planner ServiceHelper.py failed. Retrying now.")
                rate.sleep()
                continue
        
        print("Frame Converter - New pose:", new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z)
        #print(new_pose)

        return new_pose.pose

    def getJointPos(self, ref_arm_name:str,target_arm_name:str,link) -> Pose:
        """ Get target cartesian joint coordinates from reference point
        INPUT: string ref_arm_name, string target_arm_name, int link
        OUTPUT: Pose - Orientation in Euler angles not quaternions
        """
        #tfBuffer = tf2_ros.Buffer() might be done already in init
        #listener = tf2_ros.TransformListener(tfBuffer) #create transform listener 
        #rate = rospy.Rate()
        BaseID=ref_arm_name+'/base_link'
        joint_pos = Pose()
        joint_header = Header()
        linkID=target_arm_name+'/link'+str(link)
        try:
            trans = self.tfBuffer.lookup_transform(BaseID, linkID, rospy.Time(0)) # get transform between base and link0
            joint_pos.position.x=trans.transform.translation.x #unit: meters
            joint_pos.position.y=trans.transform.translation.y
            joint_pos.position.z=trans.transform.translation.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([float(trans.transform.rotation.x),float(trans.transform.rotation.y),float(trans.transform.rotation.z),float(trans.transform.rotation.w)])
            joint_pos.orientation.x=pitch
            joint_pos.orientation.y=roll
            joint_pos.orientation.z=yaw
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException): 
            rospy.loginfo("Error Transformation not found")
        return joint_pos

    def EuclidianDistance(self,x,y,xgoal,ygoal):
        d = ((x-xgoal)**2+(y-ygoal)**2)**0.5 #absolute distance
        return d

    def PotentialAttractionChange(self,x,y,xgoal,ygoal,D): #attraction at a specific point
        """ Calculates potential change from point to goal
        INPUT: current position and goal position XYs and distance where laws change. 
        OUTPUT: PotentialChange (a tuple of the change in potential along x and y axis (deltaX,deltaY))
        """
        SF = 0.2 #scaling factor
        d= self.EuclidianDistance(x,y,xgoal,ygoal)
        if d <= D:
            PotentialChange = (SF*x-SF*xgoal,SF*y-SF*ygoal)
        if d > D:
            PotentialChange = ((SF*x-SF*xgoal)/d,(SF*y-SF*ygoal)/d)
        print('attraction change:',PotentialChange,'distance:',d)
        return PotentialChange

    def PotentialAttraction(self,x,y,xgoal,ygoal,D): #the attractive field as a whole
        """ Calculates potential from point to goal
        INPUT: current position and goal position XYs and distance where laws change. 
        OUTPUT PotentialAtt (a single value for the Potential at those coordinates)
        """
        SF = 0.2 #scaling factor
        d = self.EuclidianDistance(x,y,xgoal,ygoal)
        if d <= D:
            PotentialAtt = 0.5*SF*(d**2)
        else:
            PotentialAtt = D*SF*d - 0.5*SF*D**2
        return PotentialAtt

    def PotentialRepulsion(self,x,y,xobj,yobj,Q): #Repulsive field as a whole
        """ 
        INPUT: current position and obstacle postition XYs. 
        OUTPUT: PotentialRep (a single value for Potential at those coordinates)
        """
        SF = 5000
        PotentialRep = 0
        for object in range(len(xobj)):
            d = self.EuclidianDistance(x,y,xobj[object],yobj[object])
            if d <= Q:
                PotentialRepcurrent = SF*((1/d)-(1/Q))
            else:
                PotentialRepcurrent = 0
            if PotentialRepcurrent > 100:
                PotentialRepcurrent = 100
            PotentialRep += PotentialRepcurrent
        return PotentialRep

    def PotentialRepulsionChange(self,x,y,xobj,yobj,xgoal,ygoal,Q):
        """
        INPUT: current position and obstacle position XYs. 
        OUTPUT: is repulsion at a specific point
        """
        allvectorsx = 0
        allvectorsy = 0
        repulsionangle = 0
        for object in range(len(xobj)):

            homevect = (xgoal-x,ygoal-y)
            objvect = (xobj[object]-x,yobj[object]-y)
            anglegoal = math.atan2(homevect[1],homevect[0])
            angleobj = math.atan2(objvect[1],objvect[0])
            angle = angleobj-anglegoal
            if angle > 0 or angle == 0:
                repulsionangle = anglegoal - 90
            if angle < 0:
                repulsionangle = anglegoal + 90
            d = self.EuclidianDistance(x,y,xobj[object],yobj[object])
            SF = 5*(d-Q)
            repulsionvect = SF*math.cos(angle)*math.cos(repulsionangle),SF*math.cos(angle)*math.sin(repulsionangle)
            if d > Q:
                repulsionvect = 0,0
            allvectorsx += repulsionvect[0]
            allvectorsy += repulsionvect[1]
        return allvectorsx,allvectorsy

    
    def PathPlanner(self,x,y,xgoal,ygoal,xobj,yobj,Q,D): #you are currently trying to add this in, this is the path from a point using position and force ads velocity
        """
        returned as an array of points
        INPUT: start position and goal position XYs, xobj and yobj (array of obstacle x/y points)   
        OUTPUT: PathPoints (an array of the via points ((x1,y1),(x2,y2),(x3,y3)....))
        """
        PathComplete = 0 #This turns to 1 and ends the function once end effector has reached target position (minimum of pootential)
        PathPointsx = [x] #First X and Y points
        PathPointsy = [y] #These are in different arrays cos tuples suck. The 'zip' function at the end turns them into a tuple
        i = 0
        while PathComplete == 0:
            diffattx,diffatty = self.PotentialAttractionChange(PathPointsx[i],PathPointsy[i],xgoal,ygoal,D)
            diffrepx,diffrepy = self.PotentialRepulsionChange(PathPointsx[i],PathPointsy[i],xobj,yobj,xgoal,ygoal,Q)
            difx, dify = diffattx + diffrepx,diffatty + diffrepy
            d = self.EuclidianDistance(x,y,xgoal,ygoal)
            if abs(difx) <0.2 and abs(dify) <0.2 and d < 2:#
                PathComplete = 1
            if abs(difx) < 0.1 and abs(dify) < 0.1:
                pass
                #add get out of minima here
            else:
                #print('Iteration: ',i,'x,y: ',PathPointsx,PathPointsy)
                nextx = PathPointsx[i] - 0.2*difx
                nexty = PathPointsy[i] - 0.2*dify
                x = nextx
                y = nexty
                PathPointsx.append(x)
                PathPointsy.append(y)
            i += 1
        PathPoints = list(zip(PathPointsx,PathPointsy))
        return PathPoints  

    def Space_Generation(self,startx,starty,xgoal,ygoal,xobj,yobj,Q,D): #### needs to ad objx and objy
        x = np.linspace(-50, 50, 100)  # Creating X and Y axis
        y = np.linspace(-50, 50, 100)
        X, Y = np.meshgrid(x, y)  # Creates 2 arrays with respective x any y coordination for each point
        PotentialEnergy = np.ndarray(shape=(len(x), len(y)))  # this acts as the z axis on graphs. Works better for visualisation
        for i in range(len(X)):  # gets Z values for the X Y positions
            for j in range(len(Y)):
                PotentialEnergy[i, j] = self.PotentialAttraction(X[i,j],Y[i,j],xgoal,ygoal,D)+ self.PotentialRepulsion(X[i,j],Y[i,j],xobj,yobj,Q)
                            # PotentialAttraction(X[i,j],Y[i,j],xgoal,ygoal,D) +PotentialRepulsion(X[i, j], Y[i, j], objx, objy,
        PathTaken = self.PathPlanner(startx, starty, xgoal, ygoal, xobj, yobj,Q, D)  ## you are here ^^^
        EnergyPathTaken = []
        xline = []
        yline = []
        for i in range(len(PathTaken)):
            x, y = PathTaken[i]
            xline.append(x)
            yline.append(y)
            TotalPotential = self.PotentialAttraction(x, y, xgoal, ygoal, D) + self.PotentialRepulsion(x, y, xobj, yobj, Q)
            EnergyPathTaken.append(TotalPotential)
        return X,Y, xline, yline, PotentialEnergy, EnergyPathTaken, PathTaken

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
        ax = plt.axes()
        xpoints =[]
        ypoints = []
        for point in PathTaken:
            xpoints.append(point[0])
            ypoints.append(point[1])
        ax.plot(xpoints,ypoints)
        plt.show()