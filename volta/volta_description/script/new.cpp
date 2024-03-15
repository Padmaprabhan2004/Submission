#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <cmath>

class CmdVelPublisher
{
public:
    CmdVelPublisher() : nh_("~")
    {
        subscriber_ = nh_.subscribe("/diffbot_base_controller/odom", 10, &CmdVelPublisher::odom_callback, this);

        publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("/diffbot_base_controller/cmd_vel", 30);
        timer_ = nh_.createTimer(ros::Duration(1.0 / 30), &CmdVelPublisher::publish_cmd_vel, this);
        counter_ = 0;

    }

private:
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Extract position
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        // Extract orientation (quaternion)
        qx_ = msg->pose.pose.orientation.x;
        qy_ = msg->pose.pose.orientation.y;
        qz_ = msg->pose.pose.orientation.z;
        qw_ = msg->pose.pose.orientation.w;
        // Convert quaternion to yaw
        yaw_ = atan2(2 * (qw_ * qz_ + qx_ * qy_), 1 - 2 * (qy_ * qy_ + qz_ * qz_));
    }

    void publish_cmd_vel(const ros::TimerEvent& event)
    {
        geometry_msgs::TwistStamped msg;
        msg.header.stamp = ros::Time::now();
        potential();
        d_prev_ = d_;
        msg.twist.linear.x = d_;
        msg.twist.angular.z = desired_yaw1_ - yaw_;
        publisher_.publish(msg);
    }

    void potential()
    {
        double xg = 4;
        double yg = 4;
        d_ = sqrt((x_ - xg) * (x_ - xg) + (y_ - yg) * (y_ - yg));
        if (d_ > sqrt(xg * xg + yg * yg) / 2.0)
        {
            xs_ = (xg - x_) / d_;
            ys_ = (yg - y_) / d_;
        }
        else
        {
            xs_ = (xg - x_);
            ys_ = (yg - y_);
        }

        double x1 = 2;
        double y1 = 0;
        double d3 = sqrt((x_ - x1) * (x_ - x1) + (y_ - y1) * (y_ - y1));

        desired_yaw1_ = atan2(ys_, xs_);
        if (d_ <= 0.35)
        {
            d_ = 0;
            desired_yaw1_ = yaw_;
        }
        std::cout << "d1 " << d_ << " Yaw " << desired_yaw1_ << " current_yaw " << yaw_ << std::endl;
    }

    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    ros::Timer timer_;
    size_t counter_;
    double x_, y_;
    double qx_, qy_, qz_, qw_;
    double yaw_;
    double xs_;
    double ys_;
    double p_ = 0;
    double desired_yaw1_;
    double d_;
    double d_prev_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cmd_vel_publisher");
    CmdVelPublisher cmd_vel_publisher;
    ros::spin();
    return 0;
}