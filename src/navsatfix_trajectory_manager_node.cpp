/*
    NavSatFix message visualizer on rviz using its object MarkerArray

*/

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// service definition
#include <navsatfix_trajectory_manager/SetInitialLLA.h>

// -- utilities -- 
#define ros_exception(message)                             \
    std::string dump = "\nmessage: ";                      \
    dump += (message);                                     \
    dump += "\nat line: " + std::to_string(__LINE__);      \
    dump += "\nin function: " + std::string(__FUNCTION__); \
    dump += "\nin file: " + std::string(__FILE__);         \
    ROS_ERROR("%s", dump.c_str());                         \
    throw std::runtime_error(dump.c_str());

geometry_msgs::Point GetPoint(const double x, const double y, const double z)
{
    geometry_msgs::Point p;
    p.x = x, p.y = y, p.z = z;
    return p;
}

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

    void MainLoop();
private:
    visualization_msgs::MarkerArray points_;
    ros::Publisher pub_points_;

    void InitializeMarkerArray();
    void AddNewPoint(const double x, const double y, const SolutionStatus);

    // Service server instances
    ros::ServiceServer sv_set_initial_lla_;
    ros::ServiceServer sv_add_new_marker_;

    bool SetInitialLLA(
        navsatfix_trajectory_manager::SetInitialLLA::Request &req,
        navsatfix_trajectory_manager::SetInitialLLA::Response &res
    );
    bool AddNewMarker(

    );
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navsatfix_trajectory_manager_node");
    
    NavsatfixTrajectory nst;
    nst.MainLoop();
    
    return 0;
}

// -- Class Implementations --

NavsatfixTrajectory::NavsatfixTrajectory()
{
    ros::NodeHandle nh;

    // register publisher
    pub_points_ = nh.advertise<visualization_msgs::MarkerArray>("navsatfix_trajectory", 1);

    // register services
    sv_set_initial_lla_ = nh.advertiseService(
        "set_initial_lla",
        &NavsatfixTrajectory::SetInitialLLA,
        this
    );
//    sv_add_new_marker_ = nh.

    InitializeMarkerArray();

    ROS_INFO("Following services are provided");
    ROS_INFO("$rosservice call /set_initial_lla latitude longitude altitude");
    ROS_INFO("$rosservice call /add_new_marker latitude longitude altitude");
}

void NavsatfixTrajectory::MainLoop()
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        pub_points_.publish(points_);

        ros::spinOnce();
        rate.sleep();
    }
}

void NavsatfixTrajectory::InitializeMarkerArray()
{
    visualization_msgs::Marker m;
    m.header.frame_id = "/gnss";
    m.header.stamp = ros::Time::now();
    m.ns = "navsatfix_trajectory_manager_node";
    m.type = visualization_msgs::Marker::POINTS;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.1, m.scale.y = 0.1, m.scale.z = 0.1;
    points_.markers.push_back(m);

    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            AddNewPoint(double(i), double(j), SolutionStatus((i + j) % 4));
        }
    }
}

void NavsatfixTrajectory::AddNewPoint(const double x, const double y, const SolutionStatus status)
{
    visualization_msgs::Marker &m = points_.markers[0];
    
    m.points.push_back(GetPoint(x, y, 0.1));

    std_msgs::ColorRGBA color;
    color.a = 0.9;
    switch (status)
    {
    case SolutionStatus::FIX:
        color.r = 0.0, color.g = 1.0, color.b = 0.0;
        break;
    case SolutionStatus::FLOAT:
        color.r = 1.0, color.g = 1.0, color.b = 0.0;
        break;
    case SolutionStatus::DGNSS:
        color.r = 0.0, color.g = 0.0, color.b = 1.0;
        break;
    case SolutionStatus::SINGLE:
        color.r = 1.0, color.g = 0.0, color.b = 0.0;
        break;
    default:
        ros_exception("unknown status");
        break;
    }
    m.colors.push_back(color);
}

bool NavsatfixTrajectory::SetInitialLLA(
    navsatfix_trajectory_manager::SetInitialLLA::Request &req,
    navsatfix_trajectory_manager::SetInitialLLA::Response &res
)
{
    res.status = 1;
    ROS_INFO("service is called lat : %f, lon : %f, alt : %f", req.latitude, req.longitude, req.altitude);
    return true;
}