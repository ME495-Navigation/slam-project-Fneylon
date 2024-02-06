#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {

    RCLCPP_INFO(this->get_logger(), "turtle_control!");

    // Declare parameters:
    this->declare_parameter("wheel_radius", -1.0);
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    if (wheel_radius_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "wheel_radius must be greater than 0.0");
        rclcpp::shutdown();
    }

    this->declare_parameter("track_width", -1.0);
    track_width_ = this->get_parameter("track_width").as_double();
    if (track_width_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "track_width must be greater than 0.0");
        rclcpp::shutdown();
    }

    this->declare_parameter("motor_cmd_max", -1.0);
    motor_cmd_max_ = this->get_parameter("motor_cmd_max").as_double();
    if (motor_cmd_max_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "motor_cmd_max must be greater than 0.0");
        rclcpp::shutdown();
    }

    this->declare_parameter("motor_cmd_per_rad_sec", -1.0);
    motor_cmd_per_rad_sec_ = this->get_parameter("motor_cmd_per_rad_sec").as_double();
    if (motor_cmd_per_rad_sec_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "motor_cmd_per_rad_sec must be greater than 0.0");
        rclcpp::shutdown();
    }

    this->declare_parameter("encorder_ticks_per_rad", -1.0);
    encorder_ticks_per_rad_ = this->get_parameter("encorder_ticks_per_rad").as_double();
    if (encorder_ticks_per_rad_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "encorder_ticks_per_rad must be greater than 0.0");
        rclcpp::shutdown();
    }

    this->declare_parameter("collision_radius", -1.0);
    collision_radius_ = this->get_parameter("collision_radius").as_double();
    if (collision_radius_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "collision_radius must be greater than 0.0");
        rclcpp::shutdown();
    }

    // Define Timers:
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_), std::bind(&TurtleControl::timer_callback, this));
    

    // Define Publishers:
    //  Need to send this a namespace?
    wheel_cmd_pub_ = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10); 
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Define Subscribers:
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));

    sensor_sub_ = this->create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&TurtleControl::sensor_callback, this, std::placeholders::_1));
    
  }

private:
    void timer_callback()
    {
        // RCLCPP_INFO(this->get_logger(), "timer_callback!");
        // RCLCPP_INFO(this->get_logger(), "turtle_control!");
        
    }

    void sensor_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "sensor_callback!");

        turtlelib::WheelConfiguration wc;
        wc.theta_l = double(msg->left_encoder / encorder_ticks_per_rad_);
        wc.theta_r = double(msg->right_encoder / encorder_ticks_per_rad_);

        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->now();
        // joint_state_msg.name.re
        joint_state_msg.name = {"left_wheel_joint", "right_wheel_joint"};
        joint_state_msg.position.resize(2);
        // joint_state_msg.position = {wc.theta_l, wc.theta_r};
        joint_state_msg.position[0] = wc.theta_l;
        joint_state_msg.position[1] = wc.theta_r;

        joint_state_pub_->publish(joint_state_msg);


    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        
        // Assign the variables from the message to the Twist2D object
        diff_drive_ = turtlelib::DiffDrive(wheel_radius_, track_width_);
        turtlelib::Twist2D tw;
        tw.x = msg->linear.x;
        tw.y = msg->linear.y;
        tw.omega = msg->angular.z;

        // Compute the Wheel Commands from the Twist2D object
        turtlelib::WheelConfiguration wc = diff_drive_.inverse_kinematics(tw);

        // double mcu_l = motor_cmd_per_rad_sec_ * wc.theta_l;
        // double mcu_r = motor_cmd_per_rad_sec_ * wc.theta_r;

        // // Convert to match unit of time of publishing rate
        // // wc.theta_l = wc.theta_l * 1/rate_;
        // // wc.theta_r = wc.theta_r * 1/rate_;

        // // Saturate Control Signal
        // if (mcu_l > motor_cmd_max_)
        // {
        //     mcu_l = motor_cmd_max_;
        // }
        // else if (mcu_l < -motor_cmd_max_)
        // {
        //     mcu_l = -motor_cmd_max_;
        // }

        // if (mcu_r > motor_cmd_max_)
        // {
        //     mcu_r = motor_cmd_max_;
        // }
        // else if (mcu_r < -motor_cmd_max_)
        // {
        //     mcu_r = -motor_cmd_max_;
        // }


        // Publish the Wheel Commands
        // nuturtlebot_msgs::msg::WheelCommands wc_msg;
        // wc_msg.left_velocity = mcu_l / motor_cmd_per_rad_sec_;
        // wc_msg.right_velocity = mcu_r / motor_cmd_per_rad_sec_;
        // wheel_cmd_pub_->publish(wc_msg);

        nuturtlebot_msgs::msg::WheelCommands wc_msg;
        wc_msg.left_velocity = wc.theta_l;
        wc_msg.right_velocity = wc.theta_r;
        wheel_cmd_pub_->publish(wc_msg);


    }
  
  // Initalize Publishers:
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // Initialize Subscribers:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; 
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_sub_;
  

  // Initialize Services:
  

  // Initalize Broadcasters:
 

  // Initialize Timers:
  rclcpp::TimerBase::SharedPtr timer_;
  


  // Initlaize Messages:



  // Initialize Variables:
    double wheel_radius_;
    double track_width_;
    double motor_cmd_max_;
    double motor_cmd_per_rad_sec_;
    double encorder_ticks_per_rad_;
    double collision_radius_;
    double rate_ = 250;

    turtlelib::DiffDrive diff_drive_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
