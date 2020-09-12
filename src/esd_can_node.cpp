#include <ros/ros.h>
#include "../include/esd_can/EsdCAN.hpp"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "ros_package_template");
    ros::NodeHandle nodeHandle("~");

    esd_can::EsdCAN esdCAN(nodeHandle);

    ros::spin();
    return 0;

}