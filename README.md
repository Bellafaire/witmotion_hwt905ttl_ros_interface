# WitMotion HWT905TTL ROS INTERFACE

Interface for the Witmotion HWT905TTL Inclinometer for ROS 1.
This interface is derived from the [sample code](https://github.com/WITMOTION/HWT905-TTL) provided by witmotion and is provided without warranty. 

## Limitations

This node has only been tested with the following configuration on the HWT905-TTL: 
- Baud Rate = 115200
- Transmission Rate = 100Hz

It is possible for the node to function improperly in the event that it boots at the wrong point in the serial transmission from the IMU. 
As a result it is recommended to run this node with ```respawn="True"``` in your launch file. 
This node has been tested on ROS Noetic and Melodic. 

# Usage

In order to run this node clone this repo into your ros workspace and build. 
Following the build the node can be launched by using the following code in your launch file. 

```XML
<node pkg="hwt905ttl" type="hwt905ttl_node" name="imu_driver" output="screen" respawn="true">
    <param name="port" value="/dev/your_device" />
    <param name="baud_rate" value="115200" />
</node>
```

The node will publish on the topic ```/imu``` at a rate of 100Hz fully populating sensor_msgs/Imu Message with the exception of the covariance fields. 