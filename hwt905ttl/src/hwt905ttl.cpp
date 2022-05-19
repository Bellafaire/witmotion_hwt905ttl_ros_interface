#include "hwt905ttl.h"

namespace hwt905ttl
{

    bool HWT905TTL::getData()
    {

        ret = recv_data(fd, r_buf, 44);
        if (ret == -1)
        {
            ROS_ERROR_THROTTLE(1, "uart read failed!");
            return false;
        }
        for (int i = 0; i < ret; i++)
        {
            ParseData(r_buf[i]);
        }

        bool recent_orientation = (ros::Time::now() - orientation.header.stamp) < ros::Duration(1 / publish_freq);
        bool recent_accel = (ros::Time::now() - accel.header.stamp) < ros::Duration(1 / publish_freq);
        bool recent_angular_vel = (ros::Time::now() - angular_vel.header.stamp) < ros::Duration(1 / publish_freq);
        bool pub_ready = (ros::Time::now() - last_pub) > ros::Duration(1 / publish_freq);

        if (recent_orientation && recent_accel && recent_angular_vel && pub_ready)
            pub_data();
        else
            return false;
        return true;
    }

    double HWT905TTL::getPublishFreq()
    {
        return publish_freq;
    }

    bool HWT905TTL::pub_data()
    {
        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time::now();
        last_pub = msg.header.stamp;
        msg.header.frame_id = "base_link";

        msg.orientation = orientation.quaternion;
        msg.linear_acceleration = accel.vector;
        msg.angular_velocity = angular_vel.vector;

        ROS_DEBUG("IMU Data: \n   q: %0.2f %0.2f %0.2f %0.2f \n   w: %0.2f %0.2f %0.2f \n    a: %0.2f %0.2f %0.2f",
                  orientation.quaternion.x, orientation.quaternion.y, orientation.quaternion.z, orientation.quaternion.w,
                  angular_vel.vector.x, angular_vel.vector.y, angular_vel.vector.z,
                  accel.vector.x, accel.vector.y, accel.vector.z);

        // clear the messages (and by extension the stamps) so that we're not pubbing more than we should
        orientation = geometry_msgs::QuaternionStamped();
        accel = geometry_msgs::Vector3Stamped();
        angular_vel = geometry_msgs::Vector3Stamped();

        // only pub data that is stamped recently.
        imu_pub.publish(msg);
    }

    // Constructor with global and private node handle arguments
    HWT905TTL::HWT905TTL(ros::NodeHandle n, ros::NodeHandle pn, int baud, const char *port)
    {
        ROS_INFO("Created HWT905TTL Instance on port %s at baud rate of %d", port, baud);
        bzero(r_buf, 1024);

        imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);

        // open uart, check for error
        fd = uart_open(fd, port);
        if (fd == -1)
        {
            ROS_ERROR("uart_open error");
            exit(EXIT_FAILURE);
        }

        // set baud rate and check for error
        if (uart_set(fd, baud, 8, 'N', 1) == -1)
        {
            ROS_ERROR("uart set failed!");
            exit(EXIT_FAILURE);
        }
    }

    int HWT905TTL::uart_open(int fd, const char *pathname)
    {
        fd = open(pathname, O_RDWR | O_NOCTTY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return (-1);
        }
        else
            ROS_INFO("open %s success!\n", pathname);
        if (isatty(STDIN_FILENO) == 0)
            printf("standard input is not a terminal device\n");
        else
            printf("isatty success!\n");
        return fd;
    }

    int HWT905TTL::uart_set(int fd, int nSpeed, int nBits, char nEvent, int nStop)
    {
        struct termios newtio, oldtio;
        if (tcgetattr(fd, &oldtio) != 0)
        {
            perror("SetupSerial 1");
            printf("tcgetattr( fd,&oldtio) -> %d\n", tcgetattr(fd, &oldtio));
            return -1;
        }
        bzero(&newtio, sizeof(newtio));
        newtio.c_cflag |= CLOCAL | CREAD;
        newtio.c_cflag &= ~CSIZE;
        switch (nBits)
        {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
        }
        switch (nEvent)
        {
        case 'o':
        case 'O':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'e':
        case 'E':
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'n':
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            break;
        }
        switch (nSpeed)
        {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        }
        if (nStop == 1)
            newtio.c_cflag &= ~CSTOPB;
        else if (nStop == 2)
            newtio.c_cflag |= CSTOPB;
        newtio.c_cc[VTIME] = 0;
        newtio.c_cc[VMIN] = 0;
        tcflush(fd, TCIFLUSH);

        if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
        {
            perror("com set error");
            return -1;
        }
        ROS_INFO("set done!\n");
        return 0;
    }

    int HWT905TTL::uart_close()
    {
        assert(fd);
        close(fd);

        return 0;
    }
    int HWT905TTL::send_data(int fd, char *send_buffer, int length)
    {
        length = write(fd, send_buffer, length * sizeof(unsigned char));
        return length;
    }
    int HWT905TTL::recv_data(int fd, char *recv_buffer, int length)
    {
        length = read(fd, recv_buffer, length);
        return length;
    }

    void HWT905TTL::ParseData(char chr)
    {
        static char chrBuf[100];
        static unsigned char chrCnt = 0;
        signed short sData[4];
        unsigned char i;
        char cTemp = 0;
        time_t now;
        chrBuf[chrCnt++] = chr;
        if (chrCnt < 11)
            return;
        for (i = 0; i < 10; i++)
            cTemp += chrBuf[i];
        if ((chrBuf[0] != 0x55) || ((chrBuf[1] & 0x50) != 0x50) || (cTemp != chrBuf[10]))
        {
            ROS_ERROR_THROTTLE(2, "Error:%x %x\r\n", chrBuf[0], chrBuf[1]);
            memcpy(&chrBuf[0], &chrBuf[1], 10);
            chrCnt--;

            // this is literally "Turn it off and turn it back on again" and I hate it but I can't really think of any other solutions to this problem.
            ROS_ERROR("Could not parse data from IMU, imu interface will shutdown. If respawn = true it should start right back up");
            ros::shutdown();

            return;
        }

        memcpy(&sData[0], &chrBuf[2], 8);
        switch (chrBuf[1])
        {
        case 0x51:
            for (i = 0; i < 3; i++)
                a[i] = (float)sData[i] / 32768.0 * 16.0;

            // load info into the accel message
            accel.vector.x = a[0];
            accel.vector.y = a[1];
            accel.vector.z = a[2];
            accel.header.stamp = ros::Time::now();

            // ROS_INFO("\r\nAccel: %6.3f %6.3f %6.3f ", a[0], a[1], a[2]);
            break;
        case 0x52:
            for (i = 0; i < 3; i++)
                w[i] = (float)sData[i] / 32768.0 * 2000.0;

            angular_vel.vector.x = w[0];
            angular_vel.vector.y = w[1];
            angular_vel.vector.z = w[2];
            angular_vel.header.stamp = ros::Time::now();

            // ROS_INFO("Angular Velocity:%7.3f %7.3f %7.3f ", w[0], w[1], w[2]);
            break;
        case 0x53:
            for (i = 0; i < 3; i++)
                Angle[i] = (float)sData[i] / 32768.0 * 180.0;
            // ROS_INFO("Angle:%7.3f %7.3f %7.3f ", Angle[0], Angle[1], Angle[2]);
            break;
        case 0x54:
            for (i = 0; i < 3; i++)
                h[i] = (float)sData[i];
            // ROS_INFO("Mag:%4.0f %4.0f %4.0f ", h[0], h[1], h[2]);
            break;
        case 0x59:
            for (i = 0; i < 4; i++)
                q[i] = (float)sData[i] / 32768.0;

            // datasheet isn't clear on what is where, sourcing the indexes from here https://github.com/wykxwyc/hwtimu_driver/blob/master/hwtimu/src/hwtimu.cpp
            orientation.quaternion.x = q[1];
            orientation.quaternion.y = q[2];
            orientation.quaternion.z = q[3];
            orientation.quaternion.w = q[0];

            orientation.header.stamp = ros::Time::now();

            // ROS_INFO("Quat:%4.3f %4.3f %4.3f %4.3f ", q[0], q[1], q[2], q[3]);
            break;
        }
        chrCnt = 0;
    }
}
