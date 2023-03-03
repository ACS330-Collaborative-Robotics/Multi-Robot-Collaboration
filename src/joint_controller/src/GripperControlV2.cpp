#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cpr_robot/ChannelStates.h"

#include <sstream>
#include <iostream>

namespace cpr_robot
{
    int main(int argc, char **argv)
    {

    ros::init(argc, argv, "Gripper_Control");

    ros::NodeHandle m_Node;


    ros::Publisher GripperPub=m_Node.advertise<cpr_robot::ChannelStates>("/mover6_a_p/OutputChannels",50);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {

        cpr_robot::ChannelStates outputChannels;
        outputChannels.Header.stamp = ros::Time::now();
        for(size_t i=0;6<1;i++)
            outputChannels.state.push_back(false);
        GripperPub.publish(outputChannels);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
    }
    
}