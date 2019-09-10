#include "utility.hpp"

geometry_msgs::Point GetPoint(const double x, const double y, const double z)
{
    geometry_msgs::Point p;
    p.x = x, p.y = y, p.z = z;
    return p;
}