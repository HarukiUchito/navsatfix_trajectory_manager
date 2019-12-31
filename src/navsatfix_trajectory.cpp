#include "navsatfix_trajectory.hpp"
#include "utility.hpp"
#include "lla.hpp"

NavsatfixTrajectory::NavsatfixTrajectory()
{
    ros::NodeHandle nh;

    // Get initial LLA from rosparam
    double lat, lon, alt;
    if (nh.getParam("gnss_ini_lat", lat))
        ROS_INFO("Initial latitudde: %f", lat);
    else ros_exception("initial latitude is not set on rosparam");
    if (nh.getParam("gnss_ini_lon", lon))
        ROS_INFO("Initial longitude: %f", lon);
    else ros_exception("initial longitude is not set on rosparam");
    if (nh.getParam("gnss_ini_alt", alt))
        ROS_INFO("Initial altitude: %f", alt);
    else ros_exception("initial altitude is not set on rosparam");
    initial_lla_ = LLA {lat, lon, alt};

    ros::NodeHandle nhp("~");
    std::string topic_name;
    if (nhp.getParam("navsatfix_to_be_visualized", topic_name))
        ROS_INFO("Topic to be visualized: %s", topic_name.c_str());
    else ros_exception("navsatfix topic is not specified on rosparam");
    // register subscriber/publisher
    sub_navsatfix_ = nh.subscribe(topic_name, 1, &NavsatfixTrajectory::SubCallback, this);
    std::string markers_name = nhp.getNamespace() + "_trajectory";
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>(markers_name, 1);
    InitializeMarkerArray();
}

void NavsatfixTrajectory::Spin()
{
    ros::Rate rate {10};
    while (ros::ok()) {
        pub_markers_.publish(markers_);

        ros::spinOnce();
        rate.sleep();
    }
}

void NavsatfixTrajectory::SubCallback(const sensor_msgs::NavSatFix::ConstPtr& navsatfix)
{
    sensor_msgs::NavSatFix cp = *navsatfix;
    // this should be fixed in publisher
    if (cp.latitude >= 100.0)
    {
        cp.status.status = SolutionStatus::SINGLE;
        
        auto conv = [](double org, int deglen)
        {
            auto os = std::to_string(org);
            auto deg = os.substr(0, deglen);
            auto mdm = os.substr(deglen, os.size() - deglen);
            return std::stod(deg) + std::stod(mdm) / 60.0;
        };
        
        cp.latitude = conv(cp.latitude, 2);
        cp.longitude = conv(cp.longitude, 3);
    }

    LLA current_lla {cp};

    geometry_msgs::Point pos = CalcRelativePosition(initial_lla_, current_lla);
    ROS_INFO("new point lat: %f, lon: %f -> x: %f, y: %f", cp.latitude, cp.longitude, pos.x, pos.y);
    AddNewPoint(pos.x, pos.y, SolutionStatus(cp.status.status));
}

void NavsatfixTrajectory::InitializeMarkerArray()
{
    const char* frame_id = "/gps";
    const char* name_space = "navsatfix_trajectory_manager_node";
    const double point_size = 0.2;
    const double line_size = 0.1;

    // Initialize points
    visualization_msgs::Marker points;
    points.header.frame_id = frame_id;
    points.header.stamp = ros::Time::now();
    points.ns = std::string(name_space);
    points.id = markers_.markers.size();
    points.type = visualization_msgs::Marker::POINTS;
    points.action = visualization_msgs::Marker::ADD;
    points.scale.x = point_size, points.scale.y = point_size, points.scale.z = point_size;
    markers_.markers.push_back(points);

    // Initialize lines
    visualization_msgs::Marker lines;
    lines.header.frame_id = frame_id;
    lines.header.stamp = ros::Time::now();
    lines.ns = std::string(name_space);
    lines.id = markers_.markers.size();
    lines.type = visualization_msgs::Marker::LINE_STRIP;
    lines.action = visualization_msgs::Marker::ADD;
    lines.scale.x = line_size, lines.scale.y = line_size, lines.scale.z = line_size;
    lines.color.a = 0.3, lines.color.r = 1.0, lines.color.g = 1.0, lines.color.b = 1.0;
    markers_.markers.push_back(lines);
}

void NavsatfixTrajectory::AddNewPoint(const double x, const double y, const SolutionStatus status)
{
    visualization_msgs::Marker &points_r = markers_.markers[0];
    visualization_msgs::Marker &lines_r = markers_.markers[1];

    auto pos = GetPointMsg(x, y, 0.3);
    points_r.points.push_back(pos);
    lines_r.points.push_back(pos);

    std_msgs::ColorRGBA color;
    color.a = 1.0;
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
    points_r.colors.push_back(color);
}
