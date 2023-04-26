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
        ROS_ERROR_STREAM("Play/Pause - Error, cannot use value: " << desired_state);
        return false;
    }

    // Setup pause physical robots publishers
    vector<ros::Publisher> pause_physical_pubs;
    for (auto & robot_name : robot_namespaces) {
        string pub_topic = "/" + robot_name + "/pause_physical";
        //ROS_INFO_STREAM("Play/Pause Service - Intialising: " << pub_topic);
        pause_physical_pubs.push_back(n.advertise<std_msgs::Bool>(pub_topic, 10, true));
    }
    ros::Duration(0.1).sleep();
    
    // Empty publisher to use in future
    ros::Publisher pause_physical_pub;
    std_msgs::Bool physical_state;

    if (desired_state == "play") {
        //** Resume physical robots **//

        // Update block positions from physical setup
        ROS_WARN_STREAM("Play/Pause Service - TODO: Add block position reset.");

        // Reset robot positions from physical setup
        ROS_WARN_STREAM("Play/Pause Service - TODO: Add robot position reset.");

        // Re-enable physical robots
        physical_state.data = 1;

        for (auto & robot_name : robot_namespaces) {
            string pub_topic = "/" + robot_name + "/pause_physical";
            pause_physical_pub = n.advertise<std_msgs::Bool>(pub_topic, 10, true);
            ros::Duration(0.1).sleep();
            pause_physical_pub.publish(physical_state);
            ROS_INFO_STREAM("Play/Pause Service - Published " << 1 << " to " << pub_topic);
        }

    } else {
        //** Pause  physical robots **//

        // Disable physical robots
        physical_state.data = 0;

        for (auto & robot_name : robot_namespaces) {
            string pub_topic = "/" + robot_name + "/pause_physical";
            pause_physical_pub = n.advertise<std_msgs::Bool>(pub_topic, 10, true);
            ros::Duration(0.1).sleep();
            pause_physical_pub.publish(physical_state);
            ROS_INFO_STREAM("Play/Pause Service - Published " << 0 << " to " << pub_topic);
        }
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