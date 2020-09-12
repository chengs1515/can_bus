#include <ros/ros.h>
#include "../include/esd_can/EsdCAN.hpp"
#include "../include/esd_can/CANBus.hpp"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "can_bus");
    ros::NodeHandle nodeHandle("~");

    esd_can::CANBus CANBus(nodeHandle);
    CANBus.run()
    ros::spin();
    return 0;

}