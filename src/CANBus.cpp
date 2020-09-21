#include "../include/can_bus/EsdCAN.hpp"
#include "../include/can_bus/CANBus.hpp"
#include <iostream>
#include <algorithm>
#include <ros/time.h>

#include "../include/can_bus/byte.h"

//#include "autoware_msgs/Gear.h"

using  can_bus::EsdCAN;
using std::cout;
using std::endl;

namespace can_bus{

CANBus::CANBus(ros::NodeHandle &NodeHandle):nh_(NodeHandle)
{
    can_client_ = new EsdCAN(0);
    
    cout<<"init sucessfully"<<endl;
    ctr_sub_ = nh_.subscribe("/ctrl_cmd",1,&CANBus::ctrlCallback,this);
    can_pb_ = nh_.advertise<autoware_msgs::VehicleStatus>("/vehicle_status",1);
    caninfo_pub_ = nh_.advertise<autoware_can_msgs::CANInfo>("/can_info",1);

    seq_ = 0;
    crtl_call_ = 0;
}


void CANBus::ctrlCallback(const autoware_msgs::ControlCommand& msg)
{
    ctrl_call_++;
    can_send_.clear();
    can_bus::CanFrame brake111;
    can_bus::CanFrame gear114;
    can_bus::CanFrame steer112;
    can_bus::CanFrame throttle110;
    can_bus::CanFrame turn113;
    can_bus::CanFrame undercontrol115;

    brake111.id = 0x111;
    brake111.len = 2;
    uint8_t* brake_data = brake111.data;
    gear114.id = 0x114;
    gear114.len = 1;
    uint8_t* gear_data = gear114.data;
    steer112.id = 0x112;
    steer112.len = 3;
    uint8_t* str_data = steer112.data;
    throttle110.id = 0x110;
    throttle110.len = 2;
    uint8_t* throttle_data = throttle110.data;
    turn113.id = 0x113;
    turn113.len = 1;
    uint8_t* turn_data = turn113.data;
    undercontrol115.id = 0x115;
    undercontrol115.len = 1;
    uint8_t* control_data = undercontrol115.data;

    
    double throttle_cmd, brake_cmd, gear_cmd = 0;
    double diff = msg.linear_velocity - vel_;
    double ts = 0.01;
    double calib_val = longctrl_.pidcontroller(diff,ts,vel_)

    if(calib_val>=0)
    {
        throttle_cmd = calib_val;
        brake_cmd = 0.0;
    }
    else
    {
        brake_cmd = -calib_val;
        throttle_cmd = 0.0;
    }

    throttle_cmd = std::min(throttle_cmd,100.0);
    brake_cmd = std::min(brake_cmd,100.0);
    double str_cmd = msg.steering_angle;//right为正
    boundData(-0.333,0.333,str_cmd);
    int str_cmd_int = static_cast<int>(str_cmd / 0.001000);
    uint8_t t = 0;
    int x;
    if(ctrl_call_>=400)
    {
        x = 1;
    }
    else
    {
        x = 0;
    }
    
    //设置发送命令
    Byte under_ctrl(control_data+0);
    under_ctrl.set_value(static_cast<uint8_t>(1),0,8);

    Byte throttle_en_set(throttle_data+0);
    throttle_en_set.set_value(static_cast<uint8_t>(x), 0, 8);
    Byte throttle_data_set(throttle_data+1);
    throttle_data_set.set_value(static_cast<uint8_t>(throttle_cmd), 0, 8);

    Byte brake_en_set(brake_data+0);
    brake_en_set.set_value(static_cast<uint8_t>(x), 0, 8);
    Byte brake_data_set(brake_data+1);
    brake_data_set.set_value(static_cast<uint8_t>(brake_cmd), 0, 8);

    Byte str_en_set(str_data+0);
    str_en_set.set_value(static_cast<uint8_t>(x),0,8);

    t = static_cast<uint8_t>(str_cmd_int & 0xFF);
    Byte str_data_set0(str_data + 1);
    str_data_set0.set_value(t, 0, 8);
    str_cmd_int >>= 8;

    t = static_cast<uint8_t>(str_cmd_int & 0xFF);
    Byte str_data_set1(str_data + 2);
    str_data_set1.set_value(t, 0, 8);

    // if(msg.linear_velocity<-2)
    //     gear_cmd = 2;
    // else if(msg.linear_velocity>2)
    //     gear_cmd = 4;
    // else
    //     gear_cmd = 0;
    // gear_cmd = (int)msg.linear_velocity;
    // gear_cmd = gear_cmd>=0?gear_cmd:0;
    // gear_cmd = gear_cmd<5?gear_cmd:4;
    

    Byte gear_set(gear_data+0);
    gear_set.set_value(static_cast<uint8_t>(gear_cmd), 0, 8);

    can_send_.push_back(brake111);
    can_send_.push_back(gear114);
    can_send_.push_back(steer112);
    can_send_.push_back(throttle110);
    can_send_.push_back(undercontrol115);


    can_client_->Send(can_send_,&send_frame_);
    can_send_.clear();
}

double CANBus::boundData(double low,double high,double& data)
{
    data = std::min(high,data);
    data = std::max(low,data);
}

void CANBus::timerCallback(const ros::TimerEvent& event)
{
    //std::cout<<"This is loop"<<std::endl;
    int errcount = 0;
    std::vector<CanFrame> buf;
    int32_t frame_num = MAX_CAN_RECV_FRAME_LEN;
    
    if (!can_client_->Receive(&buf, &frame_num)) 
    {
        std::cout  << "CAN Received " << ++errcount<<" error messages."<<std::endl;
        return;
    }
    errcount = 0;
    //cout<<"2"<<endl;
    if (buf.size() != static_cast<size_t>(frame_num)) {
      std::cout << "Receiver buf size [" << buf.size()
                        << "] does not match can_client returned length["
                        << frame_num << "]."<<std::endl;
    }

    if (frame_num == 0) {
        cout<< "CAN Received " << ++errcount << " empty messages."<<endl;
      return;
    }
    errcount = 0;
    //cout<<1<<endl;
    vs_.header.seq = seq_;
    can_info_.header.seq = seq_++; 
    vs_.header.stamp = ros::Time::now();
    can_info_.header.stamp = ros::Time::now();
    vs_.header.frame_id = "/can";
    can_info_.header.stamp = ros::Time::now();
    vs_.tm = "I don't know what's this used for";
    for (const auto &frame : buf) {
        uint8_t len = frame.len;
        uint32_t uid = frame.id;
        const uint8_t *data = frame.data;
        Parse(len,uid, data);
    }
    vs_.drivemode = autoware_msgs::VehicleStatus::MODE_MANUAL;
    vs_.steeringmode = autoware_msgs::VehicleStatus::MODE_MANUAL;
    can_pb_.publish(vs_);
    caninfo_pub_.publish(can_info_);

}

void CANBus::Parse(uint8_t leng,uint32_t uid,const uint8_t* data)
{
    if(uid == 0x510)
    {
        Byte t0(data+1);
        vs_.drivepedal = (int)t0.get_byte(0,8);
        can_info_.drivepedal = vs_.drivepedal;

    }
    else if(uid == 0x511)
    {
        Byte t0(data+1);
        vs_.brakepedal = (int)t0.get_byte(0,8);
        can_info_.brakepedal = vs_.brakepedal;
    }
    else if(uid == 0x512)
    {
        Byte t0(data + 2);
        int32_t x = t0.get_byte(0, 8);

        Byte t1(data + 1);
        int32_t t = t1.get_byte(0, 8);
        x <<= 8;
        x |= t;

        x <<= 16;
        x >>= 16;

        vs_.angle = x * 0.001000;// in radian
        can_info_.angle = vs_.angle;
    }
    else if(uid == 0x515)
    {
        Byte t0(data + 1);
        int32_t x = t0.get_byte(0, 8);

        Byte t1(data + 0);
        int32_t t = t1.get_byte(0, 8);
        x <<= 8;
        x |= t;

        x <<= 16;
        x >>= 16;
        vel_ = x*0.01;
        vs_.speed =  3.6*vel_;
        can_info_.speed = vs_.speed;
    }
}

}
