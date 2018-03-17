#include "dbw_node_core.h"

namespace DBWNODE_NS{

DBWNode::DBWNode()
: private_nh_("~"),
sys_enable_(false)
{
    ROS_DEBUG("initForROS...");
    initForROS();
}

DBWNode::~DBWNode()
{}

void DBWNode::initForROS()
{
    private_nh_.param<double>("vehicle_mass", vehicle_mass_, 1736.35);
    private_nh_.param<double>("fuel_capacity", fuel_capacity_, 13.5);
    private_nh_.param<double>("brake_deadband", brake_deadband_, 0.1);
    private_nh_.param<double>("decel_limit", decel_limit_, -5);
    private_nh_.param<double>("accel_limit", accel_limit_, 1.0);
    private_nh_.param<double>("wheel_radius", wheel_radius_, 0.2413);
    private_nh_.param<double>("wheel_base", wheel_base_, 2.8498);
    private_nh_.param<double>("steer_ratio", steer_ratio_, 14.8);
    private_nh_.param<double>("max_lat_accel", max_lat_accel_, 3.0);
    private_nh_.param<double>("max_steer_angle", max_steer_angle_, 8.0);

    steer_pub_ = nh_.advertise<dbw_mkz_msgs::SteeringCmd>("/vehicle/steering_cmd", 2);
    throttle_pub_ = nh_.advertise<dbw_mkz_msgs::ThrottleCmd>("/vehicle/throttle_cmd", 2);
    brake_pub_ = nh_.advertise<dbw_mkz_msgs::BrakeCmd>("/vehicle/brake_cmd", 2);

    sub_vel_ = nh_.subscribe("/twist_cmd", 2, &DBWNode::cbFromTwistCmd, this);
    sub_enable_ = nh_.subscribe("/vehicle/dbw_enabled", 2, &DBWNode::cbFromRecvEnable, this);
    sub_cur_vel_ = nh_.subscribe("/current_velocity", 2, &DBWNode::cbFromCurrentVelocity, this);

}

void DBWNode::run()
{
    ros::Rate loop_rate(LOOP_RATE); //50hz
    while(ros::ok())
    {
        ros::spinOnce();

        if(sys_enable_ == true)
        {
            
            return;
        }

        loop_rate.sleep();

    }
}


void DBWNode::publishControlCmd(float throttle, float brake, float steer)
{
    dbw_mkz_msgs::ThrottleCmd tcmd = dbw_mkz_msgs::ThrottleCmd();
    tcmd.enable = true;
    tcmd.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
    tcmd.pedal_cmd = throttle;
    this->throttle_pub_.publish(tcmd);

    dbw_mkz_msgs::SteeringCmd scmd = dbw_mkz_msgs::SteeringCmd();
    scmd.enable = true;
    scmd.steering_wheel_angle_cmd = steer;
    this->steer_pub_.publish(scmd);

    dbw_mkz_msgs::BrakeCmd bcmd = dbw_mkz_msgs::BrakeCmd();
    bcmd.enable = true;
    bcmd.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_TORQUE;
    bcmd.pedal_cmd = brake;
    this->brake_pub_.publish(bcmd);

}

void DBWNode::cbFromRecvEnable(const std_msgs::Bool::ConstPtr& msg)
{
    sys_enable_ = msg->data;
}

void DBWNode::cbFromTwistCmd(const geometry_msgs::Twist::ConstPtr& msg)
{
}

void DBWNode::cbFromCurrentVelocity(const geometry_msgs::Twist::ConstPtr& msg)
{
}



}