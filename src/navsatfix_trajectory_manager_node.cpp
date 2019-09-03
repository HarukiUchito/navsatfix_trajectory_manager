#include <ros/ros.h>

// service definition
#include <navsatfix_trajectory_manager/SetInitialLLA.h>

bool set_initial_lla(
    navsatfix_trajectory_manager::SetInitialLLA::Request &req,
    navsatfix_trajectory_manager::SetInitialLLA::Response &res
)
{
    res.status = 1;
    ROS_INFO("service is called lat : %f, lon : %f, alt : %f", req.latitude, req.longitude, req.altitude);
    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "navsatfix_trajectory_manager_node");
    
    ros::NodeHandle nh;
    ros::ServiceServer sv = nh.advertiseService("set_initial_lla", set_initial_lla);
    ROS_INFO("service is started");
    ros::spin();

    return 0;
}