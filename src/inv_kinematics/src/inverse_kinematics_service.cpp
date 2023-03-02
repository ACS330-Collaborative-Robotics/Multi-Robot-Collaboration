#include "ros/ros.h"
#include "inv_kinematics/InvKin.h"

using namespace std;

bool inverse_kinematics(inv_kinematics::InvKin::Request &req, inv_kinematics::InvKin::Response &res) {
    cout << req << "\n";
    cout << res << "\n";

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inverse_kinematics_server");
    ros::NodeHandle n;
    ROS_INFO("Inverse Kinematics Service Started.");
    

    ros::ServiceServer service = n.advertiseService("inverse_kinematics", inverse_kinematics);

    ros::spin();

    return 0;
}