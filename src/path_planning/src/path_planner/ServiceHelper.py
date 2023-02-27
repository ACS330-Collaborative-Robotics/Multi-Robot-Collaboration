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

        INPUT: geometry_msgs Pose() - Orientation in Euler angles not quaternions

        Uses inverse_kinematics service.
        """
        rospy.wait_for_service('inverse_kinematics')

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
        start_pose.header.frame_id = reference_frame
        start_pose.header.stamp = rospy.get_rostime()

        # Convert from world frame to robot frame using tf2
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                new_pose = self.tfBuffer.transform(start_pose, target_frame+"_base")
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                print("Failed")
                continue
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
        SF = 0.5 #scaling factor
        d= self.EuclidianDistance(x,y,xgoal,ygoal)
        if d <= D: #switch to more precise potential at small ranges
            PotentialChange = (SF*x-SF*xgoal,SF*y-SF*ygoal) #in x and y (simplified tuple)
        if d > D:
            PotentialChange = (D*SF*x-D*SF*xgoal/d,D*SF*y-D*SF*ygoal/d)
        return PotentialChange

    def PotentialAttraction(self,x,y,xgoal,ygoal,D): #the attractive field as a whole
        """ Calculates potential from point to goal
        INPUT: current position and goal position XYs and distance where laws change. 
        OUTPUT PotentialAtt (a single value for the Potential at those coordinates)
        """
        SF = 0.5 #scaling factor
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
        SF = 1.5
        PotentialRep = 0
        for object in range(len(xobj)):
            print(object)
            d = self.EuclidianDistance(x,y,xobj[object],yobj[object])
            if d <= Q:
                PotentialRepcurrent = 0.5*SF*((1/d)-(1/Q))**2
            else:
                PotentialRepcurrent = 0
            if PotentialRepcurrent > 100:
                PotentialRepcurrent = 100
            PotentialRep += PotentialRepcurrent
        return PotentialRep

    def PotentialRepulsionChange(self,x,y,xobj,yobj,Q): #repulsion at a specific point
        """
        INPUT: current position and obstacle position XYs. 
        OUTPUT: is repulsion at a specific point
        """
        SF = 1.5
        #print(d)
        PotentialRepChangex = 0
        PotentialRepChangey = 0
        for obj in range(len(xobj)):
            d = self.EuclidianDistance(x, y, xobj[obj], yobj[obj])
            if d <= Q: #no repulsion outside of a safe range 
                PotentialRepChangexcurrent = -SF*(1/d - 1/Q)*(x-xobj[obj]/abs(x-xobj[obj]))*1/(d**2) #'push' in x and y
                PotentialRepChangeycurrent = -SF*(1/d - 1/Q)*(y-yobj[obj]/abs(y-yobj[obj]))*1/(d**2)
                PotentialRepChangey += PotentialRepChangeycurrent
            else:
                PotentialRepChangex += 0
                PotentialRepChangey += 0
        PotentialRepChange = PotentialRepChangex, PotentialRepChangey #put into tuple
        return PotentialRepChange

    
    def APFPathPlanner(self,x,y,xgoal,ygoal,xobj,yobj,Q,D): #you are currently trying to add this in, this is the path from a point using position and force ads velocity
        """
        returned as an array of points
        INPUT: start position and goal position XYs, xobj and yobj (array of obstacle x/y points)   
        OUTPUT: PathPoints (an array of the via points ((x1,y1),(x2,y2),(x3,y3)....))
        """
        PathComplete = 0 #This turns to 1 and ends the function once end effector has reached target position (minimum of potential)
        PathPointsx = [x] #First X and Y points
        PathPointsy = [y] #Currently arrays, The 'zip' function at the end turns them into a tuple
        i = 0
        while PathComplete == 0:
            diffattx,diffatty = self.PotentialAttractionChange(PathPointsx[i],PathPointsy[i],xgoal,ygoal,D) #what is the attractive slope of the current pos
            diffrepx,diffrepy = self.PotentialRepulsionChange(PathPointsx[i],PathPointsy[i],xobj,yobj,Q) #what is the repulsive slope of current pos
            difx = diffattx+diffrepx #total force (attract and repulsive combined)
            dify = diffatty+diffrepy
            d = self.EuclidianDistance(x,y,xgoal,ygoal) #distance to objects
            #rospy.loginfo([difx,dify,d])
            if abs(difx) <0.2 and abs(dify) <0.2 and d < 0.005: #if gradient is small then goal reached (local minima issues here)
                PathComplete = 1
            if abs(difx) < 0.001 and abs(dify) <0.001 and d >0.01: #gradient small but goal not reached
                #pass
                rospy.logwarn("LOCAL MINIMA ISSUE")
                PathPoints = list(zip(PathPointsx,PathPointsy))
                return PathPoints #DELETE
                #add get out of local minima here
            else:
                #print('Iteration: ',i,'diff x,y: ',diffrepx,diffrepy)
                nextx = PathPointsx[i] - difx*0.2 #make next point (with scaling factor)
                nexty = PathPointsy[i] - dify*0.2 
                x = nextx
                y = nexty
                rospy.loginfo([x,y,d])
                PathPointsx.append(x) #add next point to list of points
                PathPointsy.append(y)
            i += 1
        PathPoints = list(zip(PathPointsx,PathPointsy)) #put into a tuple
        return PathPoints    

    def APFSpace_Generation(self,startx,starty,xgoal,ygoal,xobj,yobj,Q,D): #### needs to ad objx and objy
        x = np.linspace(-1, 1, 4)  # Creating X and Y axis
        y = np.linspace(-1, 1, 4)
        X, Y = np.meshgrid(x, y)  # Creates 2 arrays with respective x any y coordination for each point
        PotentialEnergy = np.ndarray(shape=(len(x), len(y)))  # this acts as the z axis on graphs. Works better for visualisation
        for i in range(len(X)):  # gets Z values for the X Y positions
            for j in range(len(Y)):
                PotentialEnergy[i, j] = self.PotentialAttraction(X[i,j],Y[i,j],xgoal,ygoal,D) +self.PotentialRepulsion(X[i,j],Y[i,j],xobj,yobj,Q)
                            # self.PotentialAttraction(X[i,j],Y[i,j],xgoal,ygoal,D) +self.PotentialRepulsion(X[i, j], Y[i, j], objx, objy,
        PathTaken = self.APFPathPlanner(startx, starty, xgoal, ygoal, xobj, yobj,Q, D)  ## you are here ^^^
        EnergyPathTaken = []
        xline = []
        yline = []
        for i in range(len(PathTaken)):
            x, y = PathTaken[i]
            xline.append(x)
            yline.append(y)
            TotalPotential = self.PotentialAttraction(x, y, xgoal, ygoal, D) + self.PotentialRepulsion(x, y, xobj, yobj, Q)
            EnergyPathTaken.append(TotalPotential)
        return X,Y, xline, yline, PotentialEnergy, EnergyPathTaken

    def APFplot(self,X,Y, xline, yline, PotentialEnergy,EnergyPathTaken):
        # Making 3d Plot
        fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
        ax.clear()
        ax.plot_surface(X, Y, PotentialEnergy)
        ax.plot(xline, yline, EnergyPathTaken, color='red', linewidth=4.5)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        plt.show()
        print("Successfuly run")