#include "ros/ros.h"
#include <ros/console.h>

#include <string.h>
#include <vector>

#include "std_msgs/Bool.h"
#include "custom_msgs/PlayPause.h"

using namespace std;

bool play_pause(custom_msgs::PlayPause::Request &req, custom_msgs::PlayPause::Response &res) {
    string desired_state = req.state;

    vector<string> robot_namespaces = {"mover6_a", "mover6_b"};

    ros::NodeHandle n;

    ROS_INFO_STREAM("Play/Pause Service recieved:\t" << desired_state);

    if ((desired_state != "play") && (desired_state != "pause")) {
        res.success = false;
        ROS_ERROR_STREAM("Play/Pause - Error, cannot use value:\t" << desired_state);
        return false;
    }

    // Setup pause physical robots publishers
    ros::Publisher pause_physical_pub;
    for (auto & robot_name : robot_namespaces) {
        ROS_WARN_STREAM("Play/Pause Service - Intialising: " << robot_name);
        string pub_topic = "/" + robot_name + "/pause_physical";
        ROS_WARN_STREAM(pub_topic);
        pause_physical_pub = n.advertise<std_msgs::Bool>(pub_topic, 10, true);
    }

    ros::Duration(1).sleep();

    if (desired_state == "play") {
        //** Resume physical robots **//

        // Update block positions from physical setup

        // Re-enable physical robots
        std_msgs::Bool physical_state;
        physical_state.data = 1;

        for (auto & robot_name : robot_namespaces) {
            string pub_topic = "/" + robot_name + "/pause_physical";
            pause_physical_pub = n.advertise<std_msgs::Bool>(pub_topic, 10, true);
            pause_physical_pub.publish(physical_state);
            ROS_INFO("PUB - ENABLE");
        }

    } else {
        //** Pause  physical robots **//

        // Disable physical robots
        std_msgs::Bool physical_state;
        physical_state.data = 0;

        for (auto & robot_name : robot_namespaces) {
            string pub_topic = "/" + robot_name + "/pause_physical";
            pause_physical_pub = n.advertise<std_msgs::Bool>(pub_topic, 10, true);
            pause_physical_pub.publish(physical_state);
            ROS_INFO("PUB - DISABLE");
        }
    }

    ros::Duration(1).sleep();

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