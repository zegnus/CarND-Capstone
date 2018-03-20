#include "dbw_node_core.h"

namespace DBWNODE_NS{

DBWNode::DBWNode()
: private_nh_("~"),
sys_enable_(false),
control_gap_(1.0/LOOP_RATE),
srv_(private_nh_)
{
    ROS_INFO("DBW_node launch...initForROS...");
    initForROS();
}

DBWNode::~DBWNode()
{}

void DBWNode::initForROS()
{
    srv_.setCallback(boost::bind(&DBWNode::cbFromDynamicReconfig, this, _1, _2));

    private_nh_.param<double>("vehicle_mass", vehicle_mass_, 1736.35);
    private_nh_.param<double>("fuel_capacity", fuel_capacity_, 13.5);
    private_nh_.param<double>("brake_deadband", brake_deadband_, 0.1);
    private_nh_.param<double>("decel_limit", decel_limit_, -3.0); //default: -5.0
    private_nh_.param<double>("accel_limit", accel_limit_, 3.0); //default: 1.0
    private_nh_.param<double>("wheel_radius", wheel_radius_, 0.2413);
    private_nh_.param<double>("wheel_base", wheel_base_, 2.8498);
    private_nh_.param<double>("steer_ratio", steer_ratio_, 14.8);
    private_nh_.param<double>("max_lat_accel", max_lat_accel_, 3.0);
    private_nh_.param<double>("max_steer_angle", max_steer_angle_, 8.0);

    //TODO: min_speed_
    yaw_controller_.setParameters(wheel_base_, steer_ratio_, max_lat_accel_, max_steer_angle_);

    steer_pub_ = nh_.advertise<dbw_mkz_msgs::SteeringCmd>("/vehicle/steering_cmd", 2);
    throttle_pub_ = nh_.advertise<dbw_mkz_msgs::ThrottleCmd>("/vehicle/throttle_cmd", 2);
    brake_pub_ = nh_.advertise<dbw_mkz_msgs::BrakeCmd>("/vehicle/brake_cmd", 2);

    sub_vel_ = nh_.subscribe("/twist_cmd", 2, &DBWNode::cbFromTwistCmd, this);
    sub_enable_ = nh_.subscribe("/vehicle/dbw_enabled", 2, &DBWNode::cbFromRecvEnable, this);
    sub_cur_vel_ = nh_.subscribe("/current_velocity", 2, &DBWNode::cbFromCurrentVelocity, this);
    sub_steer_report_ = nh_.subscribe("/vehicle/steering_report", 2, &DBWNode::cbFromSteeringReport, this);

    //give a general value for MKZ.
    lpf_fuel_.setParams(63.0, 0.1); 
    //assume 50% Fuel level
    lpf_fuel_.filt(50);
    //ROS_INFO("DBW_node, lpf_fuel_.get(): %f", lpf_fuel_.get());
    //ROS_INFO("DBW_node, lpf_fuel_.getReady(): %d", lpf_fuel_.getReady());

    //TODO accel_limit_
    lpf_accel_.setParams(0.5, 0.02);

    speed_pid_.setRange(decel_limit_, accel_limit_);
    accel_pid_.setRange(0, accel_limit_);
}

void DBWNode::run()
{
    ros::Rate loop_rate(LOOP_RATE); //50hz

    while(ros::ok())
    {
        ros::spinOnce();

        if(sys_enable_ == true)
        {
            getPredictedControlValues();
        }
        else
        {
            speed_pid_.resetError();
            accel_pid_.resetError();
        }

        loop_rate.sleep();

    }
}


void DBWNode::publishControlCmd(Controller v_controller)
{
    dbw_mkz_msgs::ThrottleCmd tcmd = dbw_mkz_msgs::ThrottleCmd();
    tcmd.enable = true;
    tcmd.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
    tcmd.pedal_cmd = v_controller.throttle;
    this->throttle_pub_.publish(tcmd);

    dbw_mkz_msgs::SteeringCmd scmd = dbw_mkz_msgs::SteeringCmd();
    scmd.enable = true;
    scmd.steering_wheel_angle_cmd = v_controller.steer;
    this->steer_pub_.publish(scmd);

    dbw_mkz_msgs::BrakeCmd bcmd = dbw_mkz_msgs::BrakeCmd();
    bcmd.enable = true;
    bcmd.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_TORQUE;
    bcmd.pedal_cmd = v_controller.brake;
    this->brake_pub_.publish(bcmd);

}

void DBWNode::cbFromRecvEnable(const std_msgs::Bool::ConstPtr& msg)
{
    sys_enable_ = msg->data;
}

void DBWNode::cbFromTwistCmd(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    twist_cmd_.header = msg->header;
    twist_cmd_.twist = msg->twist;
}

void DBWNode::cbFromCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    cur_velocity_.header = msg->header;
    cur_velocity_.twist = msg->twist;
}

void DBWNode::cbFromSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg)
{
    double raw_accel = LOOP_RATE * (msg->speed - cur_velocity_.twist.linear.x);
    lpf_accel_.filt(raw_accel);
}

void DBWNode::cbFromDynamicReconfig(twist_controller::ControllerConfig& config, uint32_t level)
{
    cfg_ = config;
    //set speed pid's P/I/D
    speed_pid_.setGains(cfg_.speed_kp, cfg_.speed_ki, cfg_.speed_kd);
    //set acceleration pid's P/I/D
    accel_pid_.setGains(cfg_.accel_kp, cfg_.accel_ki, cfg_.accel_kd);
}

void DBWNode::getPredictedControlValues()
{

    // vehicle mass calculation
    double vehicle_mass = vehicle_mass_ + lpf_fuel_.get() / 100.0 * fuel_capacity_ * GAS_DENSITY;

    //for simulator, for real car, should fetch from report
    double vel_cte = twist_cmd_.twist.linear.x - cur_velocity_.twist.linear.x;
    
    if(fabs(twist_cmd_.twist.linear.x) < mphToMps(1.0))
    {
        speed_pid_.resetError();
    }

    //speed pid
    double accel_cmd = speed_pid_.step(vel_cte, control_gap_);

    if(twist_cmd_.twist.linear.x <= (double)1e-2)
    {
        accel_cmd = std::min(accel_cmd, -530 / vehicle_mass / wheel_radius_);
    }

    //set throttle signal
    if(accel_cmd >= 0.0)
    {
        vehicle_controller_.throttle = accel_pid_.step(accel_cmd - lpf_accel_.get(), control_gap_);
    }
    else
    {
        accel_pid_.resetError();
        vehicle_controller_.throttle = 0.0;
    }

    //set brake signal
    if (accel_cmd < -brake_deadband_) 
    {
        vehicle_controller_.brake = -accel_cmd * vehicle_mass * wheel_base_;
        ROS_INFO("vehicle_controller_.brake: %f", vehicle_controller_.brake);
    } 
    else 
    {
        vehicle_controller_.brake = 0;
    }

    //set steering signal
    double steering_wheel = yaw_controller_.get_steering(twist_cmd_.twist.linear.x, twist_cmd_.twist.angular.z, 
                                                    cur_velocity_.twist.linear.x);
    vehicle_controller_.steer = steering_wheel + cfg_.steer_kp * (twist_cmd_.twist.angular.z - cur_velocity_.twist.angular.z);

    //TODO:: for simulator test to see work or not, remember to remove
    //vehicle_controller_.throttle = 0.06;
    //vehicle_controller_.steer = 0.01;    
    //vehicle_controller_.brake = 0;

    publishControlCmd(vehicle_controller_);

    vehicle_controller_.reset();

}

}