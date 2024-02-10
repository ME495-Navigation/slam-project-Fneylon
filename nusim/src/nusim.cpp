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
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "turtlelib/diff_drive.hpp"
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

    this->declare_parameter("wheel_radius", -1.0);
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    if (wheel_radius_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "wheel_radius must be greater than 0.0");
      rclcpp::shutdown();
    }
    this->declare_parameter("track_width", -1.0);
    track_width_ = this->get_parameter("track_width").as_double();
    if (track_width_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "track_width must be greater than 0.0");
      rclcpp::shutdown();
    }
    this->declare_parameter("motor_cmd_per_rad_sec", -1.0);
    motor_cmd_per_rad_sec_ = this->get_parameter("motor_cmd_per_rad_sec").as_double();
    if (motor_cmd_per_rad_sec_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "motor_cmd_per_rad_sec must be greater than 0.0");
      rclcpp::shutdown();
    }
    this->declare_parameter("encorder_ticks_per_rad", -1.0);
    encorder_ticks_per_rad_ = this->get_parameter("encorder_ticks_per_rad").as_double();
    if (encorder_ticks_per_rad_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "encorder_ticks_per_rad must be greater than 0.0");
      rclcpp::shutdown();
    }

    diff_drive_ = turtlelib::DiffDrive(wheel_radius_, track_width_);
    wheel_config_.theta_l = 0.0;
    wheel_config_.theta_r = 0.0;

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
    red_sensor_pub_ = this->create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);
    red_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Define the Subscribers:
    red_wheel_cmd_sub_ = this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10,
      std::bind(&Nusim::red_wheel_cmd_callback, this, std::placeholders::_1));

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
  void red_wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    // We take in wheel commands which are in mcu and we need to convert them to rad/s which are actually positions for some reason
    double left_wheel_cmd = double(msg->left_velocity);
    double right_wheel_cmd = double(msg->right_velocity);

    double left_wheel_vel = left_wheel_cmd * motor_cmd_per_rad_sec_ / rate_;
    double right_wheel_vel = right_wheel_cmd * motor_cmd_per_rad_sec_ / rate_;

    wheel_config_.theta_l += left_wheel_vel;
    wheel_config_.theta_r += right_wheel_vel;
    diff_drive_.forward_kinematics(wheel_config_);

    // We then update the transform of the robot
    update_transform(
      diff_drive_.get_configuration().x,
      diff_drive_.get_configuration().y, diff_drive_.get_configuration().theta);
    left_encoder_ = left_encoder_ + left_wheel_vel * encorder_ticks_per_rad_;
    right_encoder_ = right_encoder_ + right_wheel_vel * encorder_ticks_per_rad_;
    update_sensor_data(left_encoder_, right_encoder_);

    // Update Joint States
    sensor_msgs::msg::JointState joint_state_msg_;
    update_js(wheel_config_.theta_l, wheel_config_.theta_r);
  }

  void timer_callback()
  {
    time_ += 1.0 / rate_;
    msg_.data = (time_) * 1e3;
    publisher_->publish(msg_);

    // Update Transform with the new time and broadcast it
    update_transform();
    tf_broadcaster_->sendTransform(transformStamped_);

    // Update Sensor Data
    red_sensor_pub_->publish(sensor_msg_);

    // Update Joint States
    update_js();
    red_joint_state_pub_->publish(joint_state_msg_);

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
    update_transform(x0_, y0_, theta0_);
    tf_broadcaster_->sendTransform(transformStamped_);
  }

  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    RCLCPP_INFO(this->get_logger(), "Teleporting Red Robot!");
    update_transform(request->x, request->y, request->theta);
    tf_broadcaster_->sendTransform(transformStamped_);
  }

  void update_js()
  {
    joint_state_msg_.header.stamp = this->get_clock()->now();
    joint_state_msg_.header.frame_id = "red/base_footprint";
    joint_state_msg_.name = {"wheel_left_joint", "wheel_right_joint"};

    try {
      joint_state_msg_.position =
      {joint_state_msg_.position.at(0), joint_state_msg_.position.at(1)};
    } catch (const std::out_of_range & oor) {joint_state_msg_.position = {0.0, 0.0};}

  }

  void update_js(double left_vel, double right_vel)
  {
    joint_state_msg_.header.stamp = this->get_clock()->now();
    joint_state_msg_.header.frame_id = "red/base_footprint";
    joint_state_msg_.name = {"wheel_left_joint", "wheel_right_joint"};
    joint_state_msg_.position = {left_vel, right_vel};
  }

  void update_sensor_data(double left_vel, double right_vel)
  {
    sensor_msg_.left_encoder = left_vel;
    sensor_msg_.right_encoder = right_vel;
  }

  void update_transform(double x, double y, double theta)
  {
    transformStamped_.header.stamp = this->get_clock()->now();
    transformStamped_.header.frame_id = "nusim/world";
    transformStamped_.child_frame_id = "red/base_footprint";
    transformStamped_.transform.translation.x = x;
    transformStamped_.transform.translation.y = y;
    transformStamped_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    transformStamped_.transform.rotation.x = q.x();
    transformStamped_.transform.rotation.y = q.y();
    transformStamped_.transform.rotation.z = q.z();
    transformStamped_.transform.rotation.w = q.w();
  }

  void update_transform()
  {
    transformStamped_.header.stamp = this->get_clock()->now();
    transformStamped_.header.frame_id = "nusim/world";
    transformStamped_.child_frame_id = "red/base_footprint";
  }

  // Initalize Publishers:
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_obs_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr red_sensor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr red_joint_state_pub_;

  // Initialize Services:
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_; // Use the correct service type
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;

  // Initalize Broadcasters:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Initialize Subscribers:
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr red_wheel_cmd_sub_;

  // Initialize Timers:
  rclcpp::TimerBase::SharedPtr timer_;

  turtlelib::DiffDrive diff_drive_;

  // Initlaize Messages:
  geometry_msgs::msg::TransformStamped transformStamped_;
  std_msgs::msg::UInt64 msg_;
  nuturtlebot_msgs::msg::SensorData sensor_msg_;
  sensor_msgs::msg::JointState joint_state_msg_;

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
  double wheel_radius_;
  double track_width_;
  double motor_cmd_max_;
  double motor_cmd_per_rad_sec_;
  double encorder_ticks_per_rad_;
  double left_encoder_ = 0.0;
  double right_encoder_ = 0.0;

  turtlelib::WheelConfiguration wheel_config_; // wheel positions that are used to update the pose of the robot


};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
