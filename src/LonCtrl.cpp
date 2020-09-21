#include "../include/can_bus/LonCtrl.hpp"
#include <iostream>

using std::cout;
using std::endl;
using std::ios;


namespace can_bus
{


LonCtrl::LonCtrl()
{
    if(!readfile())
    {
        cout<<"can not read the calibration table"<<endl;
        ros::shutdown();
    }
    if(!ctr_interp_.Init(this->xyz_))
    {
        std::cout<<"interpolation failed"<<std::endl;
        ros::shutdown();
    }
}

bool LonCtrl::readfile()
{
    std::ifstream ifs;
    ifs.open("/home/vci-2019/work_ws/src/can_bus/src/calibration.txt",ios::in);
    if(!ifs)
    {
        cout<<"read calibration table failed"<<endl;
        return false;
    }
    cout<<"read calibration file sucess"<<endl;
    char buffer[1024] = {0};
    double speed = -1,acc,command;
    std::string spstr = "speed:";
    std::string accstr = "acceleration:";
    std::string comstr = "command:";
    while(ifs>>buffer)
    {
        if(buffer == spstr)
        {
            ifs>>buffer;
            speed = std::stod(buffer);
            if((ifs>>buffer)&&(buffer == accstr))
            {
                ifs>>buffer;
                acc = std::stod(buffer);
            }
            else
            {
                std::cout<<"文件格式不对，重新检查文件"<<std::endl;
                return false;
            }
            if((ifs>>buffer)&&(buffer == comstr))
            {
                ifs>>buffer;
                command = std::stod(buffer);
            }
            else
            {
                std::cout<<"文件格式不对，重新检查文件"<<std::endl;
                return false;
            }
        }
        if(speed>-0.5)
        {
            //cout<<speed*acc*command<<endl;
            auto y = std::make_tuple(speed,acc,command);
           // cout<<std::get<0>(y)<<endl;
            xyz_.push_back(std::make_tuple(speed,acc,command));
            speed = -1;
        }
    }
    return true;
}

double LonCtrl::pidcontroller(const double err,const double dt,const double vel)
{
    if(dt<=0)
    {
        cout<<"dt<=0,will use the last output, dt:"<<dt<<endl;
        return prev_ctrl_;
    }
    integral_ += err * dt * ki_;
    if (integral_ > integrator_saturation_high_) {
      integral_ = integrator_saturation_high_;
    } else if (integral_ < integrator_saturation_low_) {
      integral_ = integrator_saturation_low_;
    }
    output = err * kp_ + integral_;
    outcmd_ = ctr_interp_.Interpolate(std::make_pair(vel,output));
    prev_ctrl_ = outcmd_;
    return outcmd_;
}

}

