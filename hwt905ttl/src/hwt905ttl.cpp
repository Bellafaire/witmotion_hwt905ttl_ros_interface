#include "hwt905ttl.h"

namespace hwt905ttl
{
    // Constructor with global and private node handle arguments
    HWT905TTL::HWT905TTL(ros::NodeHandle n, ros::NodeHandle pn, int baud)
    {
        ROS_INFO("Created HWT905TTL Instance");
    }
}