#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "EsdCAN.hpp"
#include "autoware_msgs/ControlCommandStamped.h"
#include <unordered_map>
#include "Interpolation2d.hpp"
#include "autoware_msgs/VehicleStatus.h"
#include "autoware_can_msgs/CANInfo.h"
#include "LonCtrl.hpp"


namespace can_bus{


class CANBus{

public:
    CANBus(ros::NodeHandle &NodeHandle);
    void timerCallback(const ros::TimerEvent& event);

private:
    ros::NodeHandle nh_;
    can_bus::EsdCAN* can_client_;
    ros::Subscriber ctr_sub_;
    ros::Publisher ctr_pub_;
    ros::Publisher caninfo_pub_;
    std::vector<can_bus::CanFrame> can_send_;
    ros::Publisher can_pb_;
    ros::Timer timer_;
    can_bus::LonCtrl longctrl_;
    autoware_msgs::VehicleStatus vs_;
    autoware_can_msgs::CANInfo can_info_;

    
    

    double  vel_ = 0;
    double steer_transmission_ratio_ = 1;
    double steer_single_direction_max_degree_ = 0.333;
    double throttle_deadzone_ = 9;
    double brake_deadzone_ = 9;
    int32_t send_frame_ = 5;
    int seq_;
    int crtl_call_;

    void ctrlCallback(const autoware_msgs::ControlCommand& msg);
    void canSender();
    double boundData(double low,double high,double& data);
    void Parse(uint8_t leng,uint32_t uid,const uint8_t* data);
};

}