#include "ros/ros.h"
#include "std_msgs/String.h"
#include "custom_msgs/Joints.h"

#include <string>


void chatterCallback(const custom_msgs::Joints::ConstPtr& msg)
{
    ROS_INFO("[%f, %f, %f, %f, %f, %f]",msg->joints[0],msg->joints[1],msg->joints[2],msg->joints[3],msg->joints[4],msg->joints[5]);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/mover6_a/physical/joint_angles", 1000, chatterCallback);

  ros::spin();

  return 0;
}