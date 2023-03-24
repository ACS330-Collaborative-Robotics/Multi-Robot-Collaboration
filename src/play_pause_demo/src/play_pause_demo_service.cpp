#include "ros/ros.h"
#include "custom_msgs/PlayPause.h"

bool play_pause(custom_msgs::PlayPause::Request  &req, custom_msgs::PlayPause::Response &res) {
    std::cout << req;

    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "play_pause_service");

    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("play_pause_demo_service", play_pause);
    ROS_INFO("Play/Pause service started.");
    ros::spin();

    return 0;
}