#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
# include <iostream>
# include <cmath>
# include <vector>

using namespace turtlelib;

DiffDrive::DiffDrive()
{
    wheel_radius_ = 0.0;
    wheel_track_ = 0.0;

    wheel_config_.theta_l = 0.0;
    wheel_config_.theta_r = 0.0;

    config_.x = 0.0;
    config_.y = 0.0;
    config_.theta = 0.0;
}

DiffDrive::DiffDrive(double wheel_radius, double wheel_track)
{
    wheel_radius_ = wheel_radius;
    wheel_track_ = wheel_track;

    wheel_config_.theta_l = 0.0;
    wheel_config_.theta_r = 0.0;

    config_.x = 0.0;
    config_.y = 0.0;
    config_.theta = 0.0;
}

Configuration2D DiffDrive::get_configuration()
{
    return config_;
}

void DiffDrive::forward_kinematics(WheelConfiguration wheels)
{

    // Begin Citation [5]

    Configuration2D qb;
    Configuration2D dq;

    double delta_theta_l = wheels.theta_l - wheel_config_.theta_l;
    double delta_theta_r = wheels.theta_r - wheel_config_.theta_r;

    Twist2D twist;
    twist.omega = wheel_radius_*(-(delta_theta_l) / wheel_track_ + (delta_theta_r) / wheel_track_);
    twist.x = wheel_radius_*(delta_theta_l/2 + delta_theta_r/2);
    twist.y = 0.0;

    // if (twist.omega == 0.0)
    // {
    //     qb.theta = 0.0;
    //     qb.x = twist.x;
    //     qb.y = twist.y;
    // }
    // else if (twist.omega != 0.0)
    // {

    //     qb.theta = twist.omega;
    //     qb.x = (twist.x*sin(twist.omega) + twist.y*(cos(twist.omega) - 1)) / twist.omega;
    //     qb.y = (twist.y*sin(twist.omega) + twist.x*(1 - cos(twist.omega))) / twist.omega;
    // }

    // dq.theta = qb.theta;
    // dq.x = cos(config_.theta)*qb.x - sin(config_.theta)*qb.y;
    // dq.y = sin(config_.theta)*qb.x + cos(config_.theta)*qb.y;

    // Get Tbbp 
    Transform2D Tbbp;
    Tbbp = integrate_twist(twist);

    // Get Twb
    Transform2D Twb;
    Vector2D v;
    v.x = config_.x;
    v.y = config_.y;
    Twb = Transform2D(v, config_.theta);

    // Get Twbp
    Transform2D Twbp;
    Twbp = Twb*Tbbp;

    // Get changes in configuration
    double dtheta = Twbp.rotation();
    Vector2D v2;
    v2 = Twbp.translation();


    config_.theta += dtheta;
    config_.x += v2.x;
    config_.y += v2.y;

    wheel_config_.theta_l = wheels.theta_l;
    wheel_config_.theta_r = wheels.theta_r;
}

WheelConfiguration DiffDrive::inverse_kinematics(Twist2D twist)
{
    // Begin Citation [5]

    if (twist.y != 0.0)
    {
        std::cout << "Invalid Twist, y comp. non-zero" << std::endl;
    }
    else
    {

        WheelConfiguration wheels;
        wheels.theta_l = 1/wheel_radius_*(twist.x - twist.omega*wheel_track_/2);
        wheels.theta_r = 1/wheel_radius_*(twist.x + twist.omega*wheel_track_/2);

        return wheels;
    }

}