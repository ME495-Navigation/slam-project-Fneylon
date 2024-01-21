#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include "std_msgs/msg/u_int64.hpp"
// #include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp" // Include the correct header file
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"


using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim")
  {

    RCLCPP_INFO(this->get_logger(), "Starting Nusim!");

    // Declare parameters
    this->declare_parameter("rate", 200.0);
    rate_ = this->get_parameter("rate").as_double();

    this->declare_parameter("x0", 0.0);
    x0_ = this->get_parameter("x0").as_double();

    this->declare_parameter("y0", 0.0);
    y0_ = this->get_parameter("y0").as_double();

    this->declare_parameter("theta0", 0.0);
    theta0_ = this->get_parameter("theta0").as_double();
    
    
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0/rate_), std::bind(&Nusim::timer_callback, this));
    
    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    reset_srv_ = this->create_service<std_srvs::srv::Empty>("~/reset", std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2)); // Use the correct service type
    tf_broadcaster_= std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    teleport_srv_ = this->create_service<nusim::srv::Teleport>("~/teleport", std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:

  void timer_callback()
  {
    // RCLCPP_INFO(this->get_logger(), "Hello, world!");
    // time_ = this->get_clock()->now();
    time_ += 1.0/rate_;
    msg_.data = (time_)*1e3;
    publisher_->publish(msg_);


    // geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped_.header.stamp = this->get_clock()->now();
    transformStamped_.header.frame_id = "nusim/world";
    transformStamped_.child_frame_id = "red/base_footprint";
    transformStamped_.transform.translation.x = x0_;
    transformStamped_.transform.translation.y = y0_;
    transformStamped_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta0_);
    transformStamped_.transform.rotation.x = q.x();
    transformStamped_.transform.rotation.y = q.y();
    transformStamped_.transform.rotation.z = q.z();
    transformStamped_.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transformStamped_);
  }

  void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Resetting Nusim Time!");
    time_ = 0.0;
  }

  void teleport_callback(const std::shared_ptr<nusim::srv::Teleport::Request> request,
                            std::shared_ptr<nusim::srv::Teleport::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Teleporting Red Robot!");
    x0_ = request->x;
    y0_ = request->y;
    theta0_ = request->theta;
    // geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped_.header.stamp = this->get_clock()->now();
    transformStamped_.header.frame_id = "nusim/world";
    transformStamped_.child_frame_id = "red/base_footprint";
    transformStamped_.transform.translation.x = x0_;
    transformStamped_.transform.translation.y = y0_;
    transformStamped_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta0_);
    transformStamped_.transform.rotation.x = q.x();
    transformStamped_.transform.rotation.y = q.y();
    transformStamped_.transform.rotation.z = q.z();
    transformStamped_.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transformStamped_);

  }

  geometry_msgs::msg::TransformStamped transformStamped_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_; // Use the correct service type
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; 
  std_msgs::msg::UInt64 msg_;
  double time_ = 0.0;
  double rate_;
  double x0_;
  double y0_;
  double theta0_;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}