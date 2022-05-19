// ROS and node class header file
#include <ros/ros.h>
#include "hwt905ttl.h"

int main(int argc, char **argv)
{
    // Initialize ROS and declare node handles
    ros::init(argc, argv, "hwt905ttl_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    std::string port = "/dev/ttyUSB0";
    int baud_rate = 115200;

    if (!ros::param::get("~port", port))
        ROS_WARN("IMU port not set by launch, defaulting to %s", port.c_str());
    if (!ros::param::get("~baud_rate", baud_rate))
        ROS_WARN("IMU baud_rate not set by launch, defaulting to %d", baud_rate);

    // Instantiate node class
    hwt905ttl::HWT905TTL node(n, pn, baud_rate, port.c_str());

    // Spin and process callbacks
    // ros::spin();

    ros::Rate rate(node.getPublishFreq() * 2);

    unsigned long count = 0;

    while (ros::ok())
    {
        node.getData();
        rate.sleep();
        ros::spinOnce();
    }

    node.uart_close();
    ROS_INFO("HWT905TTL Interface, Closing UART");
}
