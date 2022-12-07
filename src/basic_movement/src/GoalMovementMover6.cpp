#include <ros/ros.h>
#include "std_msgs/String.h"
#include "control_msgs/JointJog.h"
#include "sensor_msgs/JointState.h"

#include <sstream>
#include <iostream>
#include <stdio.h>

/* Create node */
float joint1, joint2, joint3, joint4, joint5, joint6;
bool know_states;

void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg) {
	int i=0;
	for(std::vector<double>::const_iterator it = msg->position.begin(); it != msg->position.end(); ++it) {
		if(i==0) {
			joint1=*it;
			//*jointpos[i]=joint1;
		}
		if(i==1) {
			joint2=*it;
			//*jointpos[i]=joint2;
		}
		if(i==2) {
			joint3=*it;
			//*jointpos[i]=joint3;
		}
		if(i==3) {
			joint4=*it;
			//*jointpos[i]=joint4;
		}
		if(i==4) {
			joint5=*it;
			//*jointpos[i]=joint5;
		}
		if(i==5) {
			joint6=*it;
			//*jointpos[i]=joint6;
		}
		i++;
	}
	know_states=true;
	ROS_INFO("Receive1 State %f\t%f\t%f\t%f\t%f\t%f", joint1, joint2, joint3, joint4, joint5, joint6);
}


int main(int argc, char **argv) {
	know_states=false;
	ros::init(argc, argv, "goal_movement_example");
	ros::NodeHandle n;

	/* Create publisher to attach to JointJog */
	ros::Publisher chatter_pub = n.advertise<control_msgs::JointJog>("/JointJog",1);

	ros::Subscriber chatter_sub = n.subscribe("/joint_states", 1000, jointsCallback);

	ros::Rate loop_rate(10);

	int counter = 0;

	const char* joints[6]
        = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };

	ros::Duration(2.0).sleep();
	while(ros::ok()) {
		if(know_states) {
			float joint1_demand=0.9;
			float jointpos[6] = {joint1, joint2, joint3, joint4, joint5, joint6}; 
			for (int i=0;i<6;i++){
				if(abs(joint1_demand-jointpos[i])>0.04) {

					ROS_INFO("Setting message Go to set point");
					control_msgs::JointJog msg_start;
					std::stringstream ss;
					ss << joints[i];

					msg_start.joint_names.push_back(ss.str());
					msg_start.velocities.push_back(0.25*(joint1_demand-joint1)/abs(joint1_demand-joint1));
					msg_start.duration=5; //Unfortunately duration isn't implemented...

					ROS_INFO("Sending message");
					chatter_pub.publish(msg_start);
				}
				if(abs(joint1_demand-joint1)<0.04) {

					ROS_INFO("Setting message Stay Still");
					control_msgs::JointJog msg_start;
					std::stringstream ss;
					ss << joints[i];

					msg_start.joint_names.push_back(ss.str());
					msg_start.velocities.push_back(0);
					msg_start.duration=5; //Unfortunately duration isn't implemented...

					ROS_INFO("Sending message");
					chatter_pub.publish(msg_start);
				}
			}
		}

		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}
