/// \file circle.cpp
/// \brief This is a ROS2 node that publishes velocity commands to the turtlebot3 to make it move in a circle.
/// 
/// PARAMETERS:
///   frequency (double): The frequency at which to publish the velocity commands.
///
/// PUBLISHES:
///   cmd_vel (geometry_msgs::msg::Twist): The velocity commands to move the turtlebot3 in a circle.
///
/// SERVICES:
///   control (turtle_control::srv::Control): The service to set the velocity and radius of the circle.
///   reverse (std_srvs::srv::Empty): The service to reverse the direction of the circle.
///   stop (std_srvs::srv::Empty): The service to stop the turtlebot3.

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
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "turtle_control/srv/control.hpp"

using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {

    // Declare parameters:
    this->declare_parameter("frequency", 100.0);
    frequency_ = this->get_parameter("frequency").as_double();

    // Define Publishers:
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Define Services:
    control_srv_ = this->create_service<turtle_control::srv::Control>(
      "control",
      std::bind(&Circle::control_callback, this, std::placeholders::_1, std::placeholders::_2));

    reverse_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = this->create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(
        &Circle::stop_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    // Define the timer:
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / frequency_), std::bind(&Circle::time_callback, this));
  }

private:

  /// @brief  This function stops the turtlebot3.
  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    velocity_ = 0.0;

    geometry_msgs::msg::Twist msg;
    msg.linear.x = velocity_;
    msg.angular.z = velocity_ / radius_;
    cmd_vel_pub_->publish(msg);
  }

  /// @brief This function reverses the direction of the turtlebot3.
  void reverse_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    velocity_ = -velocity_;
  }

  /// @brief This function sets the velocity and radius of the circle.
  void control_callback(
    const std::shared_ptr<turtle_control::srv::Control::Request> request,
    const std::shared_ptr<turtle_control::srv::Control::Response>)
  {

    velocity_ = request->velocity;
    radius_ = request->radius;

  }


  /// @brief This function publishes the velocity commands to move the turtlebot3 in a circle.
  void time_callback()
  {
    if (turtlelib::almost_equal(velocity_, 0.0, 1e-12)) {
    } else {
      geometry_msgs::msg::Twist msg;
      msg.linear.x = velocity_;
      msg.angular.z = velocity_ / radius_;
      cmd_vel_pub_->publish(msg);
    }
  }
  // Initalize Publishers:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Initialize Services:
  rclcpp::Service<turtle_control::srv::Control>::SharedPtr control_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;

  // Initialize Timers:
  rclcpp::TimerBase::SharedPtr timer_;

  // Initialize Variables:
  double frequency_ = 100.0;
  double velocity_;
  double radius_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
