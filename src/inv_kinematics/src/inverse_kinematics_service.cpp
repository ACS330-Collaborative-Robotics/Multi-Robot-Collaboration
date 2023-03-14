#include "ros/ros.h"
#include "inv_kinematics/InvKin.h"
//#include <trac_ik/trac_ik.hpp>

using namespace std;

bool inverse_kinematics(inv_kinematics::InvKin::Request &req, inv_kinematics::InvKin::Response &res) {
    cout << req << "\n";
    cout << res << "\n";

    string base_link = "base_link";
    string tip_link = "link_6";
    string URDF_param="/robot_description";
    double timeout_in_secs=0.005;
    double error=1e-5; 
    //TRAC_IK::SolveType type=TRAC_IK::Speed;

    //TRAC_IK::TRAC_IK ik_solver(string base_link, string tip_link, string URDF_param="/robot_description", double timeout_in_secs=0.005, double error=1e-5, TRAC_IK::SolveType type=TRAC_IK::Speed);
    
    // NOTE: The last arguments to the constructors are optional.
    // The type can be one of the following: 
    // Speed: returns very quickly the first solution found
    // Distance: runs for the full timeout_in_secs, then returns the solution that minimizes SSE from the seed
    // Manip1: runs for full timeout, returns solution that maximizes sqrt(det(J*J^T)) (the product of the singular values of the Jacobian)
    // Manip2: runs for full timeout, returns solution that minimizes the ratio of min to max singular values of the Jacobian.

    //int rc = ik_solver.CartToJnt(KDL::JntArray joint_seed, KDL::Frame desired_end_effector_pose, KDL::JntArray& return_joints, KDL::Twist tolerances);

    // NOTE: CartToJnt succeeded if rc >=0   

    // NOTE: tolerances on the end effector pose are optional, and if not
    // provided, then by default are 0.  If given, the ABS() of the
    // values will be used to set tolerances at -tol..0..+tol for each of
    // the 6 Cartesian dimensions of the end effector pose.

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