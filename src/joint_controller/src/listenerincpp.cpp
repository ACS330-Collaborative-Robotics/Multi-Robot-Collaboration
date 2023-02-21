#include "ros/ros.h"
#include "std_msgs/String.h"
#include "custom_msgs/Joints.h"

#include <string>
#include <iostream>

using namespace std;

void chatterCallback(const custom_msgs::Joints::ConstPtr& msg)
{
    ROS_INFO("[%f, %f, %f, %f, %f, %f]",msg->joints[0],msg->joints[1],msg->joints[2],msg->joints[3],msg->joints[4],msg->joints[5]);
}

int main(int argc, char *argv[])
{
  std::string param;
  ros::init(argc, argv, "node_name");
  ros::NodeHandle nh("~");
  nh.getParam("param", param);
  ROS_INFO("Got parameter : %s", param.c_str());

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/mover6_a/physical/joint_angles", 1000, chatterCallback);

  ros::spin();

  return 0;
}