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

using namespace std::chrono_literals;

enum class State
{
    STOPPED,
    REVERSING,
    DRIVING
};


class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {

    // Declare parameters:
    this->declare_parameter("velocity", 0);
    velocity_ = this->get_parameter("velocity").as_double();


    this->declare_parameter("radius", 0);
    radius_ = this->get_parameter("radius").as_double();
    if (radius_ == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "radius not defined");
        rclcpp::shutdown();
    }

    // Define Publishers:
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 10);


    // Define Services:
    // initial_pose_srv_ = this->create_service<turtle_control::srv::InitialPose>(
    //   "~/initial_pose", std::bind(&Odometry::initial_pose_callback, this, std::placeholders::_1, std::placeholders::_2));
    

    // Define the timer: 
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_), std::bind(&Circle::time_callback, this));
  }

private:

    void time_callback()
    {
        if (state_ == State::STOPPED)
        {
            // Want to Publish a zero twist
        }
        else if (state_ == State::REVERSING)
        {
            // This is going to reverse the robot in a circle
        }
        else if (state_ == State::DRIVING)
        {
            // This is going to drive the robot in a circle.
        }

    }

  
  // Initalize Publishers:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;


  // Initialize Services:
//   rclcpp::Service<turtle_control::srv::InitialPose>::SharedPtr initial_pose_srv_;
  

  // Initialize Timers:
  rclcpp::TimerBase::SharedPtr timer_;
  


  // Initialize Variables:
   double rate_ = 100.0;
   double velocity_;
   double radius_;

   // Initialize the state:
    State state_ = State::STOPPED;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
