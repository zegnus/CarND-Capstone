#include "PIDControl.h"

namespace DBWNODE_NS{

//using namespace std;

PIDControl::PIDControl()
{
    is_initialized_ = false;
    min_num_ = std::numeric_limits<float>::min();
    max_num_ = std::numeric_limits<float>::max();
}

PIDControl::~PIDControl()
{}

void PIDControl::initial(double kp, double ki, double kd)
{
    if( is_initialized_ == false)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;

        p_error_ = i_error_ = d_error_ = 0.0;
        is_initialized_ = true;
    }

}


void PIDControl::resetError()
{
    p_error_ = i_error_ = d_error_ = 0.0;
    //is_initialized_ = false;
}

void PIDControl::updateError(double cte, double sample_time)
{
    d_error_ = (cte - p_error_) / sample_time;
    i_error_ = i_error_ + cte * sample_time;
    p_error_ = cte;

    return;
}

double PIDControl::step(double cte, double sample_time)
{
    this->updateError(cte, sample_time);

    double val = kp_* p_error_ + kd_ * d_error_ + ki_ * i_error_;

    if(val > max_num_)
    {
        val = max_num_;
    }
    else if(val < min_num_)
        val = min_num_;

    return val;

}

void PIDControl::setRange(double min, double max) 
{ 
    min_num_ = min; 
    max_num_ = max; 
}

//reset PID gains.
void PIDControl::setGains(double kp, double ki, double kd)
{
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;

        return;
}

}