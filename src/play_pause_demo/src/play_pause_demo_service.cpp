#include "ros/ros.h"
#include "custom_msgs/PlayPause.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "play_pause_service");

  ros::NodeHandle n;

  return 0;
}