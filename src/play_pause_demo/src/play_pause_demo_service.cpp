#include "ros/ros.h"
#include "custom_msgs/PlayPause.h"
#include <string.h>
#include <ros/console.h>

using namespace std;

bool play_pause(custom_msgs::PlayPause::Request &req, custom_msgs::PlayPause::Response &res) {
    string desired_state = req.state;

    ROS_INFO_STREAM("Play/Pause Service recieved:\t" << desired_state);

    if ((desired_state != "play") && (desired_state != "pause")) {
        res.success = false;
        ROS_ERROR_STREAM("Play/Pause - Error, cannot use value:\t" << desired_state);
        return false;
    }

    res.success = true;

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