#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class RplidarHumanDetection
{
public:
    /* When a human is detected within 30cm, 
    it publishes a velocity command with linear & angular velocities = 0 */
    RplidarHumanDetection() : range_min_(0.1), range_max_(3.0), stop_range_(0.3), stopped_(false)
    {
        // publish velocity commands to the ‘/cmd_vel’ topic
        cmd_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        
        //subscribe to the ‘/scan’ topic which is where the RPLIDAR sensor publishes the laser scan data
        scan_sub_ = n_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &RplidarHumanDetection::scanCallback, this);
    }

    void spin()
    {
        ros::spin();
    }

private:
    ros::NodeHandle n_;
    ros::Publisher cmd_pub_;
    ros::Subscriber scan_sub_;
    float range_min_;
    float range_max_;
    float stop_range_;
    bool stopped_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        float range;
        for (int i = 0; i < scan->ranges.size(); ++i)
        {
            range = scan->ranges[i];
            if (range > range_min_ && range < range_max_)
            {
                float angle = scan->angle_min + i * scan->angle_increment;
                if (angle > -M_PI/4 && angle < M_PI/4)
                {
                    if (range < stop_range_)
                    {
                        if (!stopped_)
                        {
                            geometry_msgs::Twist cmd;
                            cmd.linear.x = 0.0;
                            cmd.angular.z = 0.0;
                            cmd_pub_.publish(cmd);
                            ROS_INFO("Human detected within 30cm! Stopping the robot.");
                            stopped_ = true;
                        }
                    }
                    else
                    {
                        if (stopped_)
                        {
                            ROS_INFO("Human no longer within 30cm. Resuming movement.");
                            stopped_ = false;
                        }
                    }
                    return;
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_human_detection");
    RplidarHumanDetection rhd;
    rhd.spin();
    return 0;
}
