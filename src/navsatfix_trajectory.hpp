#ifndef NAVSATFIX_TRAJECTORY_HPP
#define NAVSATFIX_TRAJECTORY_HPP

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/NavSatFix.h>

#include "lla.hpp"

// -- Specilized Object definitions for this package --
enum SolutionStatus {
    FIX,
    FLOAT,
    DGNSS,
    SINGLE,
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