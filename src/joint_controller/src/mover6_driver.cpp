#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "control_msgs/JointJog.h"
#include "sensor_msgs/JointState.h"
#include "custom_msgs/Joints.h"

#include <sstream>
#include <iostream>
#include <stdio.h>

bool emergency_stop = false;


/* Create node */
float jointdemand_1, jointdemand_2, jointdemand_3, jointdemand_4, jointdemand_5, jointdemand_6;
float joint1, joint2, joint3, joint4, joint5, joint6;
bool know_states, know_demands, moving_state;

// Reciving Joint angles from robot - assingeing them to veriables - ROSINFO to terminal
void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg) {
	int i=0;
	for(std::vector<double>::const_iterator it = msg->position.begin(); it != msg->position.end(); ++it) {
		if(i==0) {
			joint1=*it;
		}
		if(i==1) {
			joint2=*it;
		}
		if(i==2) {
			joint3=*it;
		}
		if(i==3) {
			joint4=*it;
		}
		if(i==4) {
			joint5=*it;
		}
		if(i==5) {
			joint6=*it;
		}
		i++;
	}
	know_states = true;
	//ROS_INFO("Received State %f\t%f\t%f\t%f\t%f\t%f", joint1, joint2, joint3, joint4, joint5, joint6);
}


// Reviving Joint Demands from topic - assinging to veriable - ROSINFO to terminal
void listenerJointAngles(const custom_msgs::Joints::ConstPtr& msg){
	jointdemand_1=msg->joints[0];
	jointdemand_2=msg->joints[1];
	jointdemand_3=msg->joints[2];
	jointdemand_4=msg->joints[3];
	jointdemand_5=msg->joints[4];
	jointdemand_6=msg->joints[5];
	ROS_INFO("Received Goals %f\t%f\t%f\t%f\t%f\t%f", jointdemand_1, jointdemand_2, jointdemand_3, jointdemand_4, jointdemand_5, jointdemand_6);
	know_demands = true;
}

void e_stopCallback(const std_msgs::Bool::ConstPtr& msg){
	emergency_stop = msg->data;
}


int main(int argc, char **argv) {
	// setting up veriables
	know_states=false;
	know_demands=false;
	moving_state=false;
	ros::init(argc, argv, "Physical_Mover6_Driver");
	ros::NodeHandle n;
	

	// Create publishers
	ros::Publisher chatter_pub = n.advertise<control_msgs::JointJog>("JointJog",1);
	ros::Publisher moving = n.advertise<std_msgs::String>("physical/moving_state",1);

	//Creating Subscribers
	ros::Subscriber chatter_sub = n.subscribe("joint_states", 1000, jointsCallback);
	ros::Subscriber joint_demands = n.subscribe("physical/joint_angles", 10000, listenerJointAngles);
	ros::Subscriber e_stop = n.subscribe("/emergency_stop",1000, e_stopCallback);
	
	ros::Rate loop_rate(10);

	int counter = 0;

	// Joints names
	const char* joints[6]
        = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };

	ros::Duration(2.0).sleep();
	while(ros::ok()) {
		if(know_states && know_demands) {
			// Setting up lists for joints
			float joint_demands[6]= {jointdemand_1, jointdemand_2, jointdemand_3, jointdemand_4, jointdemand_5, jointdemand_6};
			float jointpos[6] = {joint1, joint2, joint3, joint4, joint5, joint6}; 

			// Accuracy of angles in radians
			float accuracy[6] = {0.04,0.04,0.04,0.04,0.05,0.05};
			
			// Gains of robots (Speeds) joint 1-6, joint 5 and 6 cant go faster
			float joint_gains[6] = {0.25, 0.25, 0.25, 0.25, 0.2, 0.1};
			for (int i=0;i<6;i++){
				//ROS_INFO("EStopState %s",emergency_stop? "True":"False");
				// Moving joints
				if(emergency_stop == true) {
					ROS_INFO("E_stop Pressed");
					control_msgs::JointJog msg_start;
					std::stringstream ss;
					ss << joints[i];

					msg_start.joint_names.push_back(ss.str());
					msg_start.velocities.push_back(0);
					msg_start.duration=5; //Unfortunately duration isn't implemented...
					chatter_pub.publish(msg_start);

				}
				else if(abs(joint_demands[i]-jointpos[i])>accuracy[i]) {

					//ROS_INFO("Setting message Go to set point point joint %d",i);
					control_msgs::JointJog msg_start;
					std::stringstream ss;
					ss << joints[i];

					msg_start.joint_names.push_back(ss.str());
					msg_start.velocities.push_back(joint_gains[i]*(joint_demands[i]-jointpos[i])/abs(joint_demands[i]-jointpos[i]));
					msg_start.duration=5; //Unfortunately duration isn't implemented...

					chatter_pub.publish(msg_start);
					moving_state = true;
				}
				// Stopping joints
				else if(abs(joint_demands[i]-jointpos[i])<accuracy[i]) {
					//ROS_INFO("Setting message Stay Still point joint %d",i);
					control_msgs::JointJog msg_start;
					std::stringstream ss;
					ss << joints[i];

					msg_start.joint_names.push_back(ss.str());
					msg_start.velocities.push_back(0);
					msg_start.duration=5; //Unfortunately duration isn't implemented...

					chatter_pub.publish(msg_start);

				}
				ros::Duration(0.01).sleep();
				
			}

			// Stating if the arm is still moving to topic
			std_msgs::String msg;
			std::stringstream ss;
			if(moving_state == true){
				ss << "Moving";
			}
			else {
				ss << "Stationery";
			}
			msg.data = ss.str();
			//ROS_INFO("%s", msg.data.c_str());
			moving.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();

		}

		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}
