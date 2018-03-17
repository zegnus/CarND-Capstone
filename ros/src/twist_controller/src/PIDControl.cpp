#include "PIDControl.h"

namespace DBWNODE_NS{

//using namespace std;

PIDControl::PIDControl()
{
}

PIDControl::PIDControl(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

PIDControl::~PIDControl()
{}




}