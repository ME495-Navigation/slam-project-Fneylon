#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <random>
#include <tuple>
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
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim")
  {

    RCLCPP_INFO(this->get_logger(), "Starting Nusim!");

    // Declare parameters
    // Declare the the timer rate parameter
    this->declare_parameter("rate", 200.0);
    rate_ = this->get_parameter("rate").as_double();

    //Declare the lidar parameters
    this->declare_parameter("basic_sensor_variance", 0.01);
    basic_sensor_variance_ = this->get_parameter("basic_sensor_variance").as_double();

    this->declare_parameter("max_range", 3.5);
    max_range_ = this->get_parameter("max_range").as_double();



    this->declare_parameter("min_lidar_range", 0.110);
    min_lidar_range_ = this->get_parameter("min_lidar_range").as_double();

    // this->declare_parameter("max_lidar_range", 3.5);
    // max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();

    this->declare_parameter("lidar_angle_increment", 0.0174533);
    lidar_angle_increment_ = this->get_parameter("lidar_angle_increment").as_double();

    this->declare_parameter("lidar_num_samps", 360);
    lidar_num_samps_ = this->get_parameter("lidar_num_samps").as_int();

    this->declare_parameter("lidar_resolution", 0.0174533);
    lidar_resolution_ = this->get_parameter("lidar_resolution").as_double();

    this->declare_parameter("lidar_noise", 0.01);
    lidar_noise_ = this->get_parameter("lidar_noise").as_double();


    // Declare the slip parameters
    this->declare_parameter("slip_fraction", 0.5);
    slip_fraction_ = this->get_parameter("slip_fraction").as_double();
    
    this->declare_parameter("input_noise", 5.0);
    input_noise_ = this->get_parameter("input_noise").as_double();

    // Declare the initial position parameters
    this->declare_parameter("x0", 0.0);
    x0_ = this->get_parameter("x0").as_double();

    this->declare_parameter("y0", 0.0);
    y0_ = this->get_parameter("y0").as_double();

    this->declare_parameter("theta0", 0.0);
    theta0_ = this->get_parameter("theta0").as_double();


    // Declare the arena parameters
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


    // Declare the wheel parameters
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

    this->declare_parameter("collision_radius", -1.0);
    coll_radius_ = this->get_parameter("collision_radius").as_double();
    if (coll_radius_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "collision_radius must be greater than 0.0");
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
    red_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

    fake_obs_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/fake_obstacles", 10);
    laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/laser_scan", 10);

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

  void timer_callback()
  {
    time_ += 1.0 / rate_;
    msg_.data = (time_) * 1e3;
    publisher_->publish(msg_);

    // Create the Fake Sensor Noise and fake lidar scan
    std::normal_distribution<double> obs_norm_distribution_(0.0, basic_sensor_variance_);
    if (turtlelib::almost_equal((time_ - time0_), 0.2, 1e-6)) {
      // Updating the measured obstacles and publishing them
      time0_ = time_;
      // Publish and update the fake markers 
      visualization_msgs::msg::MarkerArray fake_obs_array_;
      for (int i = 0; i < int(x_obstacles_.size()); ++i) {
        double diff_x = x_obstacles_.at(i) - diff_drive_.get_configuration().x;
        double diff_y = y_obstacles_.at(i) - diff_drive_.get_configuration().y;
        double theta_current = diff_drive_.get_configuration().theta;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "red/base_footprint";
        marker.ns = "fake_obstacle";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.pose.orientation.x = 1.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 0.0;
        marker.pose.position.x = diff_x * cos(theta_current) + diff_y * sin(theta_current) + obs_norm_distribution_(generator_);
        marker.pose.position.y = diff_y * cos(theta_current) - diff_x * sin(theta_current)  + obs_norm_distribution_(generator_);
        marker.pose.position.z = 0.0;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 0;
        marker.color.a = 1.0;
        marker.scale.x = radius_*2.0;
        marker.scale.y = radius_*2.0;
        marker.scale.z = 0.25;
        // dist_ = calc_distance(marker.pose.position.x, marker.pose.position.y);
        double dx = fabs(diff_drive_.get_configuration().x - x_obstacles_.at(i) + obs_norm_distribution_(generator_));
        double dy = fabs(diff_drive_.get_configuration().y -  y_obstacles_.at(i) + obs_norm_distribution_(generator_));
        dist_ = sqrt(pow(dx, 2) + pow(dy, 2));

        if (dist_ < max_range_){
          marker.action = visualization_msgs::msg::Marker::ADD;
          // fake_obs_array_.markers.push_back(marker);
        } else {
          marker.action = visualization_msgs::msg::Marker::DELETE;
          // fake_obs_array_.markers.push_back(marker);
        }
        fake_obs_array_.markers.push_back(marker);
      }
      fake_obs_pub_->publish(fake_obs_array_);

      // Set a publish the lidar data
      set_laser_msg();
      laser_scan_pub_->publish(laser_scan_msg_);

    }

    // Update Transform with the new time and broadcast it
    // RCLCPP_INFO(this->get_logger(), "Updating Transform");
    update_transform();
    tf_broadcaster_->sendTransform(transformStamped_);

    // Update Sensor Data
    // RCLCPP_INFO(this->get_logger(), "Updating Sensor Data");
    red_sensor_pub_->publish(sensor_msg_);

    // Update Joint States
    // RCLCPP_INFO(this->get_logger(), "Updating Joint States");
    update_js();
    red_joint_state_pub_->publish(joint_state_msg_);

    // Update Path
    // RCLCPP_INFO(this->get_logger(), "Updating Path");
    update_red_path();
    red_path_pub_->publish(path_msg_);

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
        marker.scale.z = 1.0;
        if (i == 0) {
          marker.pose.position.y = 0.0;
          marker.pose.position.x = -arena_x_length_ / 2.0 - dh_ / 2.0;
        } else {
          marker.pose.position.y = 0.0;
          marker.pose.position.x = arena_x_length_ / 2.0 + dh_ / 2.0;
        }
      } else if (i == 2 || i == 3) {
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1.0;

        marker.scale.x = arena_x_length_;
        marker.scale.y = dh_;
        marker.scale.z = 1.0;

        if (i == 2) {
          marker.pose.position.y = arena_y_length_ / 2.0 + dh_ / 2.0;
          marker.pose.position.x = 0.0;
        } else {
          marker.pose.position.y = -arena_y_length_ / 2.0 - dh_ / 2.0;
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
        marker.scale.x = radius_ * 2.0;
        marker.scale.y = radius_ * 2.0;
        marker.scale.z = 0.25;
        marker.pose.position.x = x_obstacles_[i];
        marker.pose.position.y = y_obstacles_[i];
        marker.pose.position.z = 0.25/2;
        marker_obstacles_array_.markers.push_back(marker);
      }
      marker_obs_pub_->publish(marker_obstacles_array_);

    } else {
      RCLCPP_INFO(this->get_logger(), "Obstacle x and y vectors are not the same size!");
      rclcpp::shutdown();
    }
}

  void set_laser_msg(){
    
    // RCLCPP_INFO(this->get_logger(), "Setting Laser Message");
    laser_scan_msg_.ranges.clear();

    laser_scan_msg_.header.stamp = this->get_clock()->now();
    laser_scan_msg_.header.frame_id = "red/base_scan";
    laser_scan_msg_.range_min = min_lidar_range_;
    laser_scan_msg_.range_max = max_range_;
    laser_scan_msg_.angle_min = 0.0;
    laser_scan_msg_.angle_max = 2 * turtlelib::PI;
    laser_scan_msg_.angle_increment = lidar_angle_increment_;
    laser_scan_msg_.time_increment = 0.000559;
    laser_scan_msg_.scan_time = 0.2; 

    // Generate LIDAR Beam
    auto x_curr = diff_drive_.get_configuration().x;
    auto y_curr = diff_drive_.get_configuration().y;

    // Define the Transform between the world frame and the robot frame (rotation and translation)
    turtlelib::Transform2D T_w_r;
    turtlelib::Vector2D v_w_r;
    v_w_r.x = x_curr;
    v_w_r.y = y_curr;
    T_w_r = turtlelib::Transform2D(v_w_r, diff_drive_.get_configuration().theta);


    for (int i = 0; i < lidar_num_samps_; i++) {
      double curr_reading = 2.0*max_range_;
      auto gamma = i * lidar_resolution_;

      // Define the Transform between the robot frame and the lidar frame (just rotation)
      turtlelib::Transform2D T_r_l;
      T_r_l = turtlelib::Transform2D(gamma);

      // Define the Transform between the lidar frame and the frame at the end of the beam (translation only in x)
      turtlelib::Transform2D T_l_b;
      turtlelib::Vector2D v_l_b;
      v_l_b.x = max_range_;
      v_l_b.y = 0.0;
      T_l_b = turtlelib::Transform2D(v_l_b);

      // Define the Transform between the world frame and the frame at the end of the beam 
      turtlelib::Transform2D T_w_b;
      T_w_b = T_w_r * T_r_l * T_l_b;

      auto x_max = T_w_b.translation().x;
      auto y_max = T_w_b.translation().y;
      auto beam = calc_line_eq(x_curr, y_curr, x_max, y_max);

      
      // Check for intersection with right wall at (x_length/2, y)
      auto x_int = arena_x_length_ / 2.0;
      auto y_int = calc_y_pt(beam, x_int);

      // Transform between the robot and the intersection point 
      turtlelib::Transform2D T_w_i;
      turtlelib::Vector2D v_w_i;
      v_w_i.x = x_int;
      v_w_i.y = y_int;
      T_w_i = turtlelib::Transform2D(v_w_i);

      // Transform the intersection point to the lidar frame
      turtlelib::Transform2D T_l_i;
      turtlelib::Transform2D T_w_l = T_w_r * T_r_l;
      T_l_i = (T_w_l.inv() * T_w_i);
      auto dist = T_l_i.translation().x; // only concerned with x component

      if (dist > 0.0){
        if (dist < curr_reading){
          curr_reading = dist;
        }
      }

      // Check for intersection with left wall at (-x_length/2, y)
      x_int = -arena_x_length_ / 2.0;
      y_int = calc_y_pt(beam, x_int);

      // Transform between the robot and the intersection point 
      v_w_i.x = x_int;
      v_w_i.y = y_int;
      T_w_i = turtlelib::Transform2D(v_w_i);

      // Transform the intersection point to the lidar frame
      T_w_l = T_w_r * T_r_l;
      T_l_i = (T_w_l.inv() * T_w_i);
      dist = T_l_i.translation().x; // only concerned with x component


      if (dist > 0.0){
        if (dist < curr_reading){
          curr_reading = dist;
        }
      } 

      // Check for intersection with top wall at (x, y_length/2)
      auto y_int_top = arena_y_length_ / 2.0;
      auto x_int_top = calc_x_pt(beam, y_int_top);
      
      // Transform between the robot and the intersection point 
      v_w_i.x = x_int_top;
      v_w_i.y = y_int_top;
      T_w_i = turtlelib::Transform2D(v_w_i);

      // Transform the intersection point to the lidar frame
      T_w_l = T_w_r * T_r_l;
      T_l_i = (T_w_l.inv()* T_w_i);
      dist = T_l_i.translation().x; // only concerned with x component

      if (dist > 0.0){
        if (dist < curr_reading){
          curr_reading = dist;
        }
      } 


      // Check for intersection with bottom wall at (x, -y_length/2)
      auto y_int_bom = - arena_y_length_ / 2.0;
      auto x_int_bom = calc_x_pt(beam, y_int_bom);
      
      // Transform between the robot and the intersection point 
      v_w_i.x = x_int_bom;
      v_w_i.y = y_int_bom;
      T_w_i = turtlelib::Transform2D(v_w_i);

      // Transform the intersection point to the lidar frame
      T_w_l = T_w_r * T_r_l;
      T_l_i = (T_w_l.inv()* T_w_i);
      dist = T_l_i.translation().x; // only concerned with x component

      if (dist > 0.0){
        if (dist < curr_reading){
          curr_reading = dist;
        }
      }


      // Detecting the Obstacles: 
      // Detecting the first obstacle
      turtlelib::Transform2D T_l_w = T_w_l.inv();
      turtlelib::Point2D obs_center;

      for (int j = 0; j < int(x_obstacles_.size()); j++) {
        obs_center.x = x_obstacles_.at(j);
        obs_center.y = y_obstacles_.at(j);

        turtlelib::Point2D lidar_obs_center = T_l_w(obs_center);

        double obs_radius = radius_;

        double x1 = std::sqrt(std::pow(obs_radius, 2) - std::pow(lidar_obs_center.y, 2)) + lidar_obs_center.x;
        double x2 = -std::sqrt(std::pow(obs_radius, 2) - std::pow(lidar_obs_center.y, 2)) + lidar_obs_center.x;

        if (x1 >= 0.0){
          if (x1 < curr_reading){
            curr_reading = x1;
          }
        }

        if (x2 >= 0.0){
          if (x2 < curr_reading){
            curr_reading = x2;
          }
        }

      }

      // Add noise to the lidar reading
      std::normal_distribution<double> norm_distribution_(0.0, lidar_noise_);
      curr_reading = curr_reading + norm_distribution_(generator_);

      if (curr_reading <= max_range_ && curr_reading >= min_lidar_range_){
        laser_scan_msg_.ranges.push_back(curr_reading);
      } else if (curr_reading > max_range_){
        laser_scan_msg_.ranges.push_back(max_range_);
      } else if (curr_reading < min_lidar_range_){
        laser_scan_msg_.ranges.push_back(min_lidar_range_);
      }

    }
  
  }

  turtlelib::LineEquation2D calc_line_eq(double x1, double y1, double x2, double y2)
  {
    turtlelib::LineEquation2D line_eq;
    line_eq.m = (y2 - y1) / (x2 - x1);
    line_eq.b = y1 - line_eq.m * x1;
    return line_eq;
  }

  double calc_x_pt(turtlelib::LineEquation2D line_eq, double y_pt)
  {
    auto x = (y_pt - line_eq.b) / line_eq.m;
    return x;

  }

  double calc_y_pt(turtlelib::LineEquation2D line_eq, double x_pt)
  {
    auto y = line_eq.m * x_pt + line_eq.b;
    return y;
  }

  
  void red_wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    // We take in wheel commands which are in mcu and we need to convert them to rad/s which are actually positions for some reason
    double left_wheel_cmd = double(msg->left_velocity) * motor_cmd_per_rad_sec_;
    double right_wheel_cmd = double(msg->right_velocity) * motor_cmd_per_rad_sec_ ;

    double vl = 0.0;
    double vr = 0.0;
    // We then add noise to the wheel commands
    std::normal_distribution<double> norm_distribution_(0.0, input_noise_);
    std::uniform_real_distribution<double> uniform_distribution_(-slip_fraction_, slip_fraction_);
    if (turtlelib::almost_equal(left_wheel_cmd, 0.0, 1e-6)){

      vl = 0.0;
    } else {
      vl = left_wheel_cmd + norm_distribution_(generator_);

      // left_wheel_cmd = left_wheel_cmd + input_noise_ * left_wheel_cmd;
    }

    if (turtlelib::almost_equal(right_wheel_cmd, 0.0, 1e-6)){

      vr = 0.0;
    } else {
      vr = right_wheel_cmd + norm_distribution_(generator_);
    }

    double left_wheel_vel = vl  / rate_;
    double right_wheel_vel = vr / rate_;

    left_encoder_ = left_encoder_ + left_wheel_vel * encorder_ticks_per_rad_;
    right_encoder_ = right_encoder_ + right_wheel_vel * encorder_ticks_per_rad_;
    update_sensor_data(left_encoder_, right_encoder_);


    // We then need to add slip
    left_wheel_vel = vl * (1.0 + uniform_distribution_(generator_));
    right_wheel_vel = vr * (1.0 + uniform_distribution_(generator_));
  
    wheel_config_.theta_l = wheel_config_.theta_l + left_wheel_vel / rate_;
    wheel_config_.theta_r = wheel_config_.theta_r + right_wheel_vel /  rate_;
    diff_drive_.forward_kinematics(wheel_config_);

    // Update Joint States
    sensor_msgs::msg::JointState joint_state_msg_;
    update_js(wheel_config_.theta_l, wheel_config_.theta_r);

    turtlelib::Configuration2D config = diff_drive_.get_configuration();
    detect_collision(config);



    // We then update the transform of the robot
    update_transform(
      diff_drive_.get_configuration().x,
      diff_drive_.get_configuration().y, diff_drive_.get_configuration().theta);

  }

  void detect_collision(turtlelib::Configuration2D config)
  {
    int count = 0;
    // RCLCPP_INFO_STREAM(this->get_logger(), "num obstacles: " << x_obstacles_.size());
    for (int i = 0; i < int(x_obstacles_.size()); i++) {

      auto dx = config.x - x_obstacles_.at(i);
      auto dy = config.y - y_obstacles_.at(i);
      auto dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));


      if (dist <= (coll_radius_ + radius_)) {

        // Compute the Line between the robot center and the obstacle
        turtlelib::Vector2D center_line;
        center_line.x = x_obstacles_.at(i) - config.x;
        center_line.y = y_obstacles_.at(i) - config.y;

        auto unit_vec = turtlelib::normalize(center_line);

        auto new_x = x_obstacles_.at(i) - unit_vec.x * (coll_radius_ + radius_);
        auto new_y = y_obstacles_.at(i) - unit_vec.y * (coll_radius_ + radius_);

        diff_drive_.set_configuration(new_x, new_y, config.theta);

      } else {
        count = count;
      }

      
    }

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

  void update_red_path()
  {
    path_msg_.header.stamp = this->get_clock()->now();
    path_msg_.header.frame_id = "nusim/world";

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = "nusim/world";
    pose.pose.position.x = diff_drive_.get_configuration().x;
    pose.pose.position.y = diff_drive_.get_configuration().y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, diff_drive_.get_configuration().theta);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    path_msg_.poses.push_back(pose);
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


  // Initalize Publishers:
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_obs_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr red_sensor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr red_joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr red_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_obs_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;

  // Initialize Services:
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_; // Use the correct service type
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;

  // Initalize Broadcasters:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Initialize Subscribers:
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr red_wheel_cmd_sub_;

  // Initialize Timers:
  rclcpp::TimerBase::SharedPtr timer_;

  // Initialize the DiffDrive Object:
  turtlelib::DiffDrive diff_drive_;

  // Initlaize Messages:
  geometry_msgs::msg::TransformStamped transformStamped_;
  std_msgs::msg::UInt64 msg_;
  nuturtlebot_msgs::msg::SensorData sensor_msg_;
  sensor_msgs::msg::JointState joint_state_msg_;
  nav_msgs::msg::Path path_msg_;
  nav_msgs::msg::Path blue_path_msg_;
  sensor_msgs::msg::LaserScan laser_scan_msg_;
  
  // Initialize Variables:
  std::vector<double> x_obstacles_;
  std::vector<double> y_obstacles_;
  double time_ = 0.0;
  double time0_ = 0.0;
  double rate_;
  double x0_;
  double y0_;
  double theta0_;
  double arena_x_length_;
  double arena_y_length_;
  double dh_ = 0.1;
  double radius_;
  double wheel_radius_;
  double track_width_;
  double motor_cmd_max_;
  double motor_cmd_per_rad_sec_;
  double encorder_ticks_per_rad_;
  double left_encoder_ = 0.0;
  double right_encoder_ = 0.0;
  double input_noise_;
  double slip_fraction_;
  double basic_sensor_variance_;
  // double max_radius_;
  double dist_;
  double coll_radius_;

  // Lidar Variables 
  double min_lidar_range_;
  double max_range_;
  double lidar_angle_increment_;
  int lidar_num_samps_;
  double lidar_resolution_;
  double lidar_noise_;
  double laser_hz_ = 1800.0;


  turtlelib::WheelConfiguration wheel_config_; // wheel positions that are used to update the pose of the robot

  // Initailize the distributions for the noise
  std::default_random_engine generator_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
