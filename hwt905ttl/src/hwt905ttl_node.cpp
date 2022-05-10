// ROS and node class header file
#include <ros/ros.h>
#include "hwt905ttl.h"

int main(int argc, char **argv)
{
    // Initialize ROS and declare node handles
    ros::init(argc, argv, "HWT905TTL");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Instantiate node class
    hwt905ttl::HWT905TTL node(n, pn, 10);

    // Spin and process callbacks
    ros::spin();
}
