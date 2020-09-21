#pragma once

#include <fstream>
#include <ros/ros.h>
#include "Interpolation2d.hpp"

namespace can_bus
{

class LonCtrl
{
public:
    LonCtrl();
    ~LonCtrl() = default;
    double pidcontroller(const double err,const double dt,const double vel);
private:
    double prev_ctrl_ = 0.0;
    double integral_ = 0.0;
    double kp_ = 0.5,ki_ = 0.1;
    double output = 0.0;
    double integrator_saturation_high_ = 0.3;
    double integrator_saturation_low_ = -0.3;
    double outcmd_ = 0.0;
    Interpolation2d ctr_interp_;
    Interpolation2d::DataType xyz_;

    bool readfile();
    double calcmd(const double targetacc,const double vel);

};
} 
