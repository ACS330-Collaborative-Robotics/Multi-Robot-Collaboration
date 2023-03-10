#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // process the distance data
    for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < 0.3) {
            // human detected within 30cm
            ROS_INFO("Human detected within 30cm!");
        }
    }
}

int main(int argc, char **argv)
{
    // initialize the ROS node
    ros::init(argc, argv, "sensor_range");
    ros::NodeHandle nh;

    // subscribe to the LIDAR sensor data
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, laserCallback);

    // spin the ROS node
    ros::spin();

    return 0;
}