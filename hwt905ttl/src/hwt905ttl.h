#ifndef HWT905TTL_H
#define HWT905TTL_H

#include <ros/ros.h>

// depends for the serial communication.
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include "sensor_msgs/Imu.h"

// Namespace matches ROS package name
namespace hwt905ttl
{
    class HWT905TTL
    {
    public:
        HWT905TTL(ros::NodeHandle n, ros::NodeHandle pn, int baud, const char *port);
        int uart_open(int fd, const char *pathname);
        int uart_set(int fd, int nSpeed, int nBits, char nEvent, int nStop);
        int uart_close();
        int send_data(int fd, char *send_buffer, int length);
        int recv_data(int fd, char *recv_buffer, int length);
        void ParseData(char chr);
        bool getData();
    private:
        double a[3], w[3], Angle[3], h[3], q[4];
        char r_buf[1024];
        int ret, fd; 
        ros::Publisher imu_pub; 
    };
}

#endif
