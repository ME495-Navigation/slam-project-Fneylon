#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim")
  
  
    RCLCPP_INFO(this->get_logger(), "Starting Nusim!");
    this->declare_parameter("frequency", 200.0);

    double frequency = this->get_parameter("frequency").as_double();
    auto msecs = std::chrono::duration<double>(1.0/frequency);
    
    timer_ = this->create_wall_timer(
      msecs, std::bind(&Nusim::timer_callback, this));
  }

  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Hello, world!");

  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}