#include "yaw_controller.h"

using namespace DBWNODE_NS;

using namespace std;

YawController::YawController()
{}

YawController::~YawController()
{}

double YawController::get_angle(double radius)
{
    double angle = atan(this->wheel_base_ / radius) * this->steer_ratio_;
    return max(min_angle_, min(max_angle_, angle));
}

double YawController::get_steering(double linear_velocity, double angular_velocity, double current_velocity)
{
    double angular_vel = 0.0;
    if(fabs(linear_velocity) > 0.0)
    {
        angular_vel = current_velocity * angular_velocity / linear_velocity;
    }
	
	if(fabs(current_velocity) > 0.1)
	{
		double max_yaw_rate = fabs(max_lat_accel_ / current_velocity);
		angular_vel = max(-max_yaw_rate, min(max_yaw_rate, angular_vel));
	}
	
	if(fabs(angular_vel) > 0.0)
	{
		return get_angle(max(current_velocity, min_speed_) / angular_vel);
	}
	else
		return 0.0;


}

void YawController::setWheelBase(double wheel_base)
{
    wheel_base_ = wheel_base;
}
    
void YawController::setSteeringRatio(double steer_ratio)
{
    steer_ratio_ = steer_ratio;
}

void YawController::setParameters(double wheel_base, double steer_ratio, double max_lat_accel, double max_steer_angle)
{
    wheel_base_ = wheel_base;
    steer_ratio_ = steer_ratio;
    //TODO
    //min_speed_ = min_speed;
    max_lat_accel_ = max_lat_accel;
    min_angle_ = -max_steer_angle;
    max_angle_ = max_steer_angle;
}

