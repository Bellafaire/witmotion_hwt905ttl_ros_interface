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
    hwt905ttl::HWT905TTL node(n, pn, 115200, "/dev/ttyUSB0");

    // Spin and process callbacks
    // ros::spin();

    ros::Rate(1 / 10.0);

    while(ros::ok())
        node.getData();

    node.uart_close();
    ROS_INFO("HWT905TTL Interface, Closing UART");
    
}
