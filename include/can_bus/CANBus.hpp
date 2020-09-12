#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "EsdCAN.hpp"
#include <autoware_msgs/ControlCommandStamped.h>
#include <unordered_map>
#include "Interpolation2d.hpp"
#include "autoware_msgs/VehicleStatus.h"
#include "autoware_can_msgs/CANInfo.h"


namespace can_bus{


class CANBus{

public:
    CANBus(ros::NodeHandle &NodeHandle);

private:
    ros::NodeHandle nh_;
    std::unique_ptr<can_bus::EsdCAN> can_client_;
    ros::Subscriber ctr_sub_;
    ros::Publisher ctr_pub_;
    ros::Publisher caninfo_pub_;
    std::vector<can_bus::CanFrame> can_send_;
    ros::Publisher can_pb_;
    ros::Timer timer_;

    autoware_msgs::VehicleStatus vs_;
    autoware_can_msgs::CANInfo can_info_;

    Interpolation2d::DataType xyz_;
    std::unique_ptr<Interpolation2d> ctr_interp_;

    double  vel_ = 0;
    double steer_transmission_ratio_ = 1;
    double steer_single_direction_max_degree_ = 0.52359877559;
    double throttle_deadzone_ = 9;
    double brake_deadzone_ = 9;

    void ctrlCallback(const autoware_msgs::ControlCommand& msg);
    void timerCallback();
    void canSender();
    bool readfile();
    void run();
    double boundData(double low,double high,double& data);
    
};

}