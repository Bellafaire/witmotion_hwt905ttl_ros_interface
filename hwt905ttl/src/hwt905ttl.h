#ifndef HWT905TTL_H
#define HWT905TTL_H

#include <ros/ros.h>

// Namespace matches ROS package name
namespace hwt905ttl
{
    class HWT905TTL
    {
    public:
        HWT905TTL(ros::NodeHandle n, ros::NodeHandle pn, int baud);
    };
}

#endif
