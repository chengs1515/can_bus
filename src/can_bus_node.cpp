#include <ros/ros.h>
#include "../include/can_bus/EsdCAN.hpp"
#include "../include/can_bus/CANBus.hpp"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "can_bus");
    ros::NodeHandle nodeHandle("~");

    can_bus::CANBus CANbus(nodeHandle);
    ros::Timer timer_ = nodeHandle.createTimer(ros::Duration(0.01),&can_bus::CANBus::timerCallback,&CANbus);
    ros::spin();
    return 0;

}