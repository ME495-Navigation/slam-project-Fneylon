#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


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
    }

    this->declare_parameter("track_width", -1.0);
    track_width_ = this->get_parameter("track_width").as_double();
    if (track_width_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "track_width must be greater than 0.0");
    }

    this->declare_parameter("motor_cmd_max", -1.0);
    motor_cmd_max_ = this->get_parameter("motor_cmd_max").as_double();
    if (motor_cmd_max_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "motor_cmd_max must be greater than 0.0");
    }

    this->declare_parameter("motor_cmd_per_rad_sec", -1.0);
    motor_cmd_per_rad_sec_ = this->get_parameter("motor_cmd_per_rad_sec").as_double();
    if (motor_cmd_per_rad_sec_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "motor_cmd_per_rad_sec must be greater than 0.0");
    }

    this->declare_parameter("encorder_ticks_per_rad", -1.0);
    encorder_ticks_per_rad_ = this->get_parameter("encorder_ticks_per_rad").as_double();
    if (encorder_ticks_per_rad_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "encorder_ticks_per_rad must be greater than 0.0");
    }

    this->declare_parameter("collision_radius", -1.0);
    collision_radius_ = this->get_parameter("collision_radius").as_double();
    if (collision_radius_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "collision_radius must be greater than 0.0");
    }

    // Define Timers:
    

    // Define Publishers:
    

    // Define Services:s
    

    // Define Broadcasters:
    
  }

private:
  
  // Initalize Publishers:
  

  // Initialize Services:
  

  // Initalize Broadcasters:
 

  // Initialize Timers:
  


  // Initlaize Messages:


  // Initialize Variables:
    double wheel_radius_;
    double track_width_;
    double motor_cmd_max_;
    double motor_cmd_per_rad_sec_;
    double encorder_ticks_per_rad_;
    double collision_radius_;


};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
