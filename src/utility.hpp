#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <geometry_msgs/Point.h>

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



#endif