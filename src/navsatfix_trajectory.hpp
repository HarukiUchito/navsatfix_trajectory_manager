#ifndef NAVSATFIX_TRAJECTORY_HPP
#define NAVSATFIX_TRAJECTORY_HPP

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/NavSatFix.h>

// service definition
#include <navsatfix_trajectory_manager/SetInitialLLA.h>

// -- utilities -- 
#define ros_exception(message)                             \
{   std::string dump = "\nmessage: ";                      \
    dump += (message);                                     \
    dump += "\nat line: " + std::to_string(__LINE__);      \
    dump += "\nin function: " + std::string(__FUNCTION__); \
    dump += "\nin file: " + std::string(__FILE__);         \
    ROS_ERROR("%s", dump.c_str());                         \
    throw std::runtime_error(dump.c_str());                \
}

geometry_msgs::Point GetPoint(const double x, const double y, const double z);

// -- Specilized Object definitions for this package --
enum SolutionStatus {
    FIX,
    FLOAT,
    DGNSS,
    SINGLE,
};

struct LLA {
    double latitude;
    double longitude;
    double altitude;
};

class NavsatfixTrajectory {
public:
    NavsatfixTrajectory();

    void Spin();
private:
    // Navsatfix related stuff to be visualized as a trajectory
    ros::Subscriber sub_navsatfix_;
    void SubCallback(const sensor_msgs::NavSatFix::ConstPtr&);

    // Marker array Publisher related stuff
    visualization_msgs::MarkerArray markers_;
    ros::Publisher pub_markers_;

    void InitializeMarkerArray();
    void AddNewPoint(const double x, const double y, const SolutionStatus);

    // Initial LLA is required for calculating relative position,
    // Therefore the exception is thrown when the node starts working without setting this.
    LLA initial_lla_;
};

#endif