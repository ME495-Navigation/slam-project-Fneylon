#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


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

    this->declare_parameter("arena_x_length", 10.0);
    arena_x_length_ = this->get_parameter("arena_x_length").as_double();

    this->declare_parameter("arena_y_length", 10.0);
    arena_y_length_ = this->get_parameter("arena_y_length").as_double();

    // std::vector<double> obstacles;
    this->declare_parameter("obstacles/x", std::vector<double>());
    x_obstacles_ = this->get_parameter("obstacles/x").as_double_array();

    this->declare_parameter("obstacles/y", std::vector<double>());
    y_obstacles_ = this->get_parameter("obstacles/y").as_double_array();

    this->declare_parameter("obstacles/r", 0.1);
    radius_ = this->get_parameter("obstacles/r").as_double();

    rclcpp::QoS marker_qos(10);
    marker_qos.transient_local();

    // Define Timers:
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_), std::bind(&Nusim::timer_callback, this));

    // Define Publishers:
    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", marker_qos);
    marker_obs_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      marker_qos);

    // Define the Subscribers:
    // red_wheel_cmd_sub_= this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>("red/wheel_cmd", 10, std::bind(&Nusim::red_wheel_cmd_callback, this, std::placeholders::_1));

    // Define Services:
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    teleport_srv_ = this->create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Define Broadcasters:
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void timer_callback()
  {
    // RCLCPP_INFO(this->get_logger(), "Hello, world!");
    // time_ = this->get_clock()->now();
    time_ += 1.0 / rate_;
    msg_.data = (time_) * 1e3;
    publisher_->publish(msg_);
    // marker_pub_->publish(marker_walls_array_);


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

    visualization_msgs::msg::MarkerArray marker_walls_array_;

    for (int i = 0; i < 4; ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "nusim/world";
      marker.ns = "wall" + std::to_string(i);
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.x = 1.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 0.0;
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
      marker.color.a = 1.0;

      if (i == 0 || i == 1) {
        marker.scale.x = dh_;
        marker.scale.y = arena_y_length_;
        marker.scale.z = 0.25;
        if (i == 0) {
          marker.pose.position.y = 0.0;
          marker.pose.position.x = -arena_x_length_ / 2 - dh_ / 2;
        } else {
          marker.pose.position.y = 0.0;
          marker.pose.position.x = arena_x_length_ / 2 + dh_ / 2;
        }
      } else if (i == 2 || i == 3) {
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1.0;

        marker.scale.x = arena_x_length_;
        marker.scale.y = dh_;
        marker.scale.z = 0.25;

        if (i == 2) {
          marker.pose.position.y = arena_y_length_ / 2 + dh_ / 2;
          marker.pose.position.x = 0.0;
        } else {
          marker.pose.position.y = -arena_y_length_ / 2 - dh_ / 2;
          marker.pose.position.x = 0.0;
        }
      }
      marker_walls_array_.markers.push_back(marker);
    }
    marker_pub_->publish(marker_walls_array_);
    int x_num_obstacles = int(x_obstacles_.size());
    int y_num_obstacles = int(y_obstacles_.size());

    if (x_num_obstacles == y_num_obstacles) {
      visualization_msgs::msg::MarkerArray marker_obstacles_array_;
      for (int i = 0; i < int(x_obstacles_.size()); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "nusim/world";
        marker.ns = "obstacle" + std::to_string(i);
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.x = 1.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 0.0;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1.0;
        marker.scale.x = radius_;
        marker.scale.y = radius_;
        marker.scale.z = 0.25;
        marker.pose.position.x = x_obstacles_[i];
        marker.pose.position.y = y_obstacles_[i];
        marker.pose.position.z = 0.0;
        marker_obstacles_array_.markers.push_back(marker);
      }
      marker_obs_pub_->publish(marker_obstacles_array_);

    } else {
      RCLCPP_INFO(this->get_logger(), "Obstacle x and y vectors are not the same size!");
      rclcpp::shutdown();
    }
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(this->get_logger(), "Resetting Nusim!");
    time_ = 0.0;
    x0_ = 0.0;
    y0_ = 0.0;
    theta0_ = 0.0;
  }

  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    RCLCPP_INFO(this->get_logger(), "Teleporting Red Robot!");
    x0_ = request->x;
    y0_ = request->y;
    theta0_ = request->theta;
  }

  // Initalize Publishers:
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_obs_pub_;

  // Initialize Services:
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_; // Use the correct service type
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;

  // Initalize Broadcasters:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Initialize Subscribers:

  // Initialize Timers:
  rclcpp::TimerBase::SharedPtr timer_;


  // Initlaize Messages:
  geometry_msgs::msg::TransformStamped transformStamped_;
  std_msgs::msg::UInt64 msg_;

  // Initialize Variables:
  double time_ = 0.0;
  double rate_;
  double x0_;
  double y0_;
  double theta0_;
  double arena_x_length_;
  double arena_y_length_;
  double dh_ = 0.1;
  std::vector<double> x_obstacles_;
  std::vector<double> y_obstacles_;
  double radius_;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
