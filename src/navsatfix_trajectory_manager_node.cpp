/*
    NavSatFix message visualizer on rviz using its object MarkerArray

    This package
        provides SetInitialLLA message
            to set initial position (latitude, longitude, altitude) for origin in rviz.
        subscribes NavSatFix message.
        publishes MarkerArray message as rviz object.
*/

#include <ros/ros.h>
#include "navsatfix_trajectory.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navsatfix_trajectory_manager_node");
    
    NavsatfixTrajectory nst;
    nst.Spin();
    
    return 0;
}

// -- Class Implementations --
