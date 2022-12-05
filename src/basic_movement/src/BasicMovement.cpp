#include <ros/ros.h>
#include "std_msgs/String.h"
#include "control_msgs/JointJog.h"

#include <sstream>
/* Create node */
int main(int argc, char **argv) {
	ros::init(argc, argv, "simple_movement_example");
	ros::NodeHandle n;
	
	/* Create publisher to attach to JointJog */
	ros::Publisher chatter_pub = n.advertise<control_msgs::JointJog>("/JointJog",1);
	
	ros::Rate loop_rate(10);
	
	int counter = 0;
	
	

	while(ros::ok()) {
		ros::Duration(2.0).sleep();
		ROS_INFO("Counter value %d", counter);

		for (int i=1;i<3;i++){
			if(counter%2==0) {
				ROS_INFO("Setting up positive message");
				control_msgs::JointJog msg_start;
				std::stringstream ss;
				ss << "joint"+i;

				msg_start.joint_names.push_back(ss.str());
				msg_start.velocities.push_back(0.5);
				msg_start.duration=5; //Unfortunately duration isn't implemented... 
				ROS_INFO("Sending message");
				chatter_pub.publish(msg_start);
				ros::spinOnce();
				loop_rate.sleep();
				
			}
			if(counter%2==1) {
				ROS_INFO("Setting up  negative message");
				control_msgs::JointJog msg_start;
				std::stringstream ss;
				ss << "joint"+i;

				msg_start.joint_names.push_back(ss.str());
				msg_start.velocities.push_back(-0.5);
				msg_start.duration=5; //Unfortunately duration isn't implemented... 
				ROS_INFO("Sending message");
				chatter_pub.publish(msg_start);
				ros::spinOnce();
				loop_rate.sleep();
			}			
		}
		
		
		ros::Duration(5.0).sleep();

		for (int i=0;i<6;i++){
			if(counter%2==1 || counter%2==0) {
				ROS_INFO("Setting up stop message");
				control_msgs::JointJog msg_stop;
				std::stringstream ss;
				ss << "joint"+i;

				msg_stop.joint_names.push_back(ss.str());
				msg_stop.velocities.push_back(0);
				msg_stop.duration=5; //Unfortunately duration isn't implemented... 
				ROS_INFO("Sending message");
				chatter_pub.publish(msg_stop);
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
		counter+=1;
	}
	return 0;
}
