#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "std_msgs/msg/u_int64.hpp"
// #include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp" // Include the correct header file

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim")
  {

    RCLCPP_INFO(this->get_logger(), "Starting Nusim!");
    this->declare_parameter("rate_", 200.0);

    rate_ = this->get_parameter("rate_").as_double();
    
    
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0/rate_), std::bind(&Nusim::timer_callback, this));
    
    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    reset_srv_ = this->create_service<std_srvs::srv::Empty>("~/reset", std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2)); // Use the correct service type
  }
private:

  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Hello, world!");
    // time_ = this->get_clock()->now();
    time_ += 1.0/rate_;
    msg_.data = (time_)*1e3;
    publisher_->publish(msg_);
  }

  void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Resetting Nusim Time!");
    time_ = 0.0;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_; // Use the correct service type
  double rate_;
  std_msgs::msg::UInt64 msg_;
  double time_ = 0.0;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}