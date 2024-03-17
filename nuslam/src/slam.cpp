/// \file slam.cpp
/// \brief A ROS node that implements a Kalman filter for a differential drive robot
///
/// PARAMETERS:
///   body_id (string): The name of the robot's body frame
///   odom_id (string): The name of the robot's odometry frame
/// PUBILSHES:
///   /green/odom (nav_msgs::msg::Odometry): The robot's odometry
///   /green/odom_path (nav_msgs::msg::Path): The robot's path
///   /green/obstacles (visualization_msgs::msg::MarkerArray): The robot's observed obstacles
/// SUBSCRIBES:
///   /odom (nav_msgs::msg::Odometry): The robot's odometry
///   /fake_obstacles (visualization_msgs::msg::MarkerArray): The robot's observed obstacles
/// SERVICES:
///   None
/// CLIENTS:
///   None

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <armadillo>

using namespace std::chrono_literals;

class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
  {
    // Declare parameters:
    this->declare_parameter("body_id", " ");
    body_id_ = this->get_parameter("body_id").as_string();
    if (body_id_ == " ") {
      RCLCPP_ERROR(this->get_logger(), "body_id must be a string");
      rclcpp::shutdown();
    }

    this->declare_parameter("odom_id", " ");
    odom_id_ = this->get_parameter("odom_id").as_string();
    if (odom_id_ == " ") {
      RCLCPP_ERROR(this->get_logger(), "odom_id must be a string");
      rclcpp::shutdown();
    }

    // Define Publishers:
    green_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/green/odom", 10);
    green_odom_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/green/odom_path", 10);
    green_marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/green/obstacles", 10);

    // Subscribe to odom topic:
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&Slam::odom_callback, this, std::placeholders::_1));

    // Subscribe to fake_obstacles topic:
    // fake_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    //   "/fake_obstacles", 10,
    //   std::bind(&Slam::fake_obstacles_callback, this, std::placeholders::_1));

    // Subscribe to reg_obstacles topic:
    reg_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/landmarks", 10,
      std::bind(&Slam::reg_obstacles_callback, this, std::placeholders::_1));

    // Define Broadcaster:
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Define the timer:
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / 5.0), std::bind(&Slam::timer_callback, this));

  }

private:
  // ROS Callbacks:
  /// \brief A callback function to update the robot's odometry and obstacle estimation based on SLAM estimation
  void timer_callback()
  {

    // Set the Map to Robot Transform:
    set_Tmr();

    // Solve for the Map to Odom Transform:
    solve_for_Tmo();

    // Set Green Odom Transform:
    set_green_odom_transform();

    // Update the Green Odom Message:
    update_green_odom_msg();

    // Update the Green Odom Path:
    update_green_odom_path();

    // Update the Green Marker Array:
    update_green_marker_msg();

    // Publish the Green Marker Array:
    green_marker_array_pub_->publish(green_marker_array_msg_);

    // Broadcast the green odom transform:
    broadcaster_->sendTransform(green_odom_tf_);

    // Set the map odom transform:
    set_Tmo_transform();

    // Broadcast the map odom transform:
    broadcaster_->sendTransform(map_odom_tf_);

  }

  /// \brief A callback function to set the current odom position of the robot
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    // Set Green Odom Message:
    green_odom_msg_ = *msg;

    // Calculate the change in x, y, and theta:
    double dx = green_odom_msg_.pose.pose.position.x - prev_green_odom_.pose.pose.position.x;
    double dy = green_odom_msg_.pose.pose.position.y - prev_green_odom_.pose.pose.position.y;
    double dtheta = turtlelib::normalize_angle(
      return_yaw(
        green_odom_msg_.pose.pose.orientation) -
      return_yaw(prev_green_odom_.pose.pose.orientation));

    // Predict the next state:
    predict(dx, dy, dtheta);

    // Calculate the Odom to Robot Transform:
    set_Tor();

    // Set the previous green odom:
    prev_green_odom_ = *msg;
  }

  /// \brief A callback function to update the robot's observed obstacles
  void reg_obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    auto marker_array = *msg;
    int num_updates = 0;

    // Define the for loop to call the update over:
    for (int i = 0; i < int(marker_array.markers.size()); i++) {
      // RCLCPP_INFO_STREAM(this->get_logger(), "BEFORE seen_obs_: " << size(seen_obs_));
      // Extract the id, x, and y from the message:
      // int idx = msg->markers.at(i).id;
      std::vector<int> temp_seen_obs = seen_obs_;
      temp_seen_obs.push_back(seen_obs_.size());

      int idx = temp_seen_obs.size() - 1;
      double min_dist = dist_threshold_;

      arma::vec loc = {msg->markers.at(i).pose.position.x, msg->markers.at(i).pose.position.y};

      arma::vec zi = cart_to_polar(loc);
      zi.at(1) = turtlelib::normalize_angle(zi.at(1));

      double x = msg->markers.at(i).pose.position.x;
      double y = msg->markers.at(i).pose.position.y;

      for (int k = 0; k < int(seen_obs_.size()); k++) {
        // Calculate Hk:
        double dx = mu_bar_.at(3 + (2 * seen_obs_.at(k))) - mu_bar_.at(1);
        double dy = mu_bar_.at(4 + (2 * seen_obs_.at(k))) - mu_bar_.at(2);

        arma::mat Hj = arma::zeros(2, 9);
        double d = pow(dx, 2) + pow(dy, 2);
        Hj.at(0, 0) = 0.0;
        Hj.at(0, 1) = -dx / sqrt(d);
        Hj.at(0, 2) = -dy / sqrt(d);

        Hj.at(1, 0) = -1.0;
        Hj.at(1, 1) = dy / d;
        Hj.at(1, 2) = -dx / d;


        Hj.at(0, 3 + (2 * k)) = dx / sqrt(d);
        Hj.at(0, 4 + (2 * k)) = dy / sqrt(d);

        Hj.at(1, 3 + (2 * k)) = -dy / d;
        Hj.at(1, 4 + (2 * k)) = dx / d;

        // Calculate the Covariance:
        // RCLCPP_INFO_STREAM(this->get_logger(), "BEFORE Sigma_bar_: " << size(Sigma_bar_));
        arma::mat Psi = Hj * Sigma_bar_ * Hj.t() + R_;
        // RCLCPP_INFO_STREAM(this->get_logger(), "AFTER Sigma_bar_: " << size(Sigma_bar_));


        // Calculate zk_hat:
        arma::vec zk = cart_to_polar({dx, dy});
        zk.at(1) = turtlelib::normalize_angle(
          turtlelib::normalize_angle(
            zk.at(
              1)) - turtlelib::normalize_angle(mu_bar_.at(0)));

        // Calculate the Mahalanobis distance:
        arma::vec diff = (zi - zk);
        diff.at(1) = turtlelib::normalize_angle(diff.at(1));
        // RCLCPP_INFO_STREAM(this->get_logger(), "diff: " << diff);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Sigma: " << Sigma);
        // RCLCPP_INFO_STREAM(this->get_logger(), "dist:" << diff.t() * Sigma.i() * diff);

        arma::vec dist = diff.t() * Psi.i() * diff;
        // auto dist = diff.t() * Sigma.i() * diff;
        // double dist = 0.0;
        // Update the minimum distance and index:
        if (dist.at(0) < min_dist) {
          min_dist = dist.at(0);
          idx = seen_obs_.at(k);
        }
      }

      // Call the update function:
      if (is_observed(idx) == true) {
        update(idx, x, y);
        num_updates++;

      } else {
        seen_obs_.push_back(idx);
        mu_bar_.at(3 + (2 * idx)) = x + mu_bar_.at(1);
        mu_bar_.at(4 + (2 * idx)) = y + mu_bar_.at(2);
        mu_.at(3 + (2 * idx)) = mu_bar_.at(3 + (2 * idx));
        mu_.at(4 + (2 * idx)) = mu_bar_.at(4 + (2 * idx));
      }
    }

    if (num_updates == 0) {
      mu_ = mu_bar_;
      Sigma_ = Sigma_bar_;
    }


  }

  /// \brief A callback function to update the robot's observed obstacles
  void fake_obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {

    auto marker_array = *msg;
    int num_updates = 0;

    // Define the for loop to call the update over:
    for (int i = 0; i < int(marker_array.markers.size()); i++) {

      // Extract the id, x, and y from the message:
      int idx = msg->markers.at(i).id;
      double x = msg->markers.at(i).pose.position.x;
      double y = msg->markers.at(i).pose.position.y;

      // Call the update function:
      if (msg->markers.at(i).action == visualization_msgs::msg::Marker::ADD) {
        if (is_observed(idx) == true) {
          update(idx, x, y);
          num_updates++;
        } else {

          seen_obs_.push_back(idx);
          mu_bar_.at(3 + (2 * idx)) = x + mu_bar_.at(1);
          mu_bar_.at(4 + (2 * idx)) = y + mu_bar_.at(2);
          mu_.at(3 + (2 * idx)) = mu_bar_.at(3 + (2 * idx));
          mu_.at(4 + (2 * idx)) = mu_bar_.at(4 + (2 * idx));
        }
      }
    }

    if (num_updates == 0) {
      mu_ = mu_bar_;
      Sigma_ = Sigma_bar_;
    }
  }


  // Kalman filter functions:
  /// \brief A function to predict the next state of the robot
  /// \param dx - the change in x
  /// \param dy - the change in y
  /// \param dtheta - the change in theta
  void predict(double dx, double dy, double dtheta)
  {
    // Calculate mu_bar:
    mu_bar_.at(0) = mu_bar_.at(0) + dtheta;
    mu_bar_.at(1) = mu_bar_.at(1) + dx;
    mu_bar_.at(2) = mu_bar_.at(2) + dy;

    mu_bar_.at(3) = mu_bar_.at(3);
    mu_bar_.at(4) = mu_bar_.at(4);
    mu_bar_.at(5) = mu_bar_.at(5);
    mu_bar_.at(6) = mu_bar_.at(6);
    mu_bar_.at(7) = mu_bar_.at(7);
    mu_bar_.at(8) = mu_bar_.at(8);

    // Calculate At_:
    calculate_A(dx, dy);

    // Calculate Q_bar_:
    calculate_Q_bar(q_);

    // Calculate Sigma_bar_:
    calculate_Sigma_bar();

  }


  /// \brief A function to convert polar to cartesian coordinates
  /// \param range
  /// \param angle
  /// \return turtlelib::Point2D
  turtlelib::Point2D polar_to_cartesian(double range, double angle)
  {
    // Convert polar to cartesian coordinates in the robot frame:
    double x = range * cos(angle);
    double y = range * sin(angle);
    turtlelib::Point2D point;
    point.x = x;
    point.y = y;
    return point;
  }

  /// @brief A function that updates the prediction based on the observed obstacles
  /// @param idx
  /// @param x
  /// @param y
  void update(int idx, double x, double y)
  {

    // Calculate z_hat:
    arma::vec z_hat = calculate_z_hat({x, y});

    // Calculate z:
    arma::vec z = calculate_z(idx);

    // Calculate the difference between the observed and predicted x and y:
    double dx = mu_bar_.at(3 + (2 * idx)) - mu_bar_.at(1);
    double dy = mu_bar_.at(4 + (2 * idx)) - mu_bar_.at(2);

    // Calculate the H matrix:
    calculate_Hj(idx, dx, dy);

    // Calculate the Kalman Gain:
    calculate_K();

    // Update mu_:
    update_mu(z, z_hat);

    // Update Sigma_:
    update_Sigma();
    mu_bar_ = mu_;
    Sigma_bar_ = Sigma_;
  }

  // Kalman Filter Helper Functions:
  /// \brief A function to update the Sigma matrix
  void update_Sigma()
  {
    Sigma_ = (arma::eye(9, 9) - (Kt_ * Hj_)) * Sigma_bar_;
  }

  /// \brief A function to update the mu matrix
  void update_mu(arma::vec z, arma::vec z_hat)
  {
    arma::vec z_diff = z_hat - z;
    z_diff.at(1) = turtlelib::normalize_angle(z_diff.at(1));
    mu_ = mu_bar_ + (Kt_ * (z_diff));
  }

  /// \brief A function to calculate the Kalman Gain
  void calculate_K()
  {

    // calculate R:
    calculate_R();

    // calculate V_:
    calculate_V();

    Kt_ = arma::zeros(9, 2);
    Kt_ = Sigma_bar_ * Hj_.t() * ((Hj_ * Sigma_bar_ * Hj_.t()) + (V_ * R_ * V_.t())).i();

  }

  /// @brief  A function to calculate the Hj matrix
  /// @param idx Obstacle index
  /// @param dx relative distance in x
  /// @param dy relative distance in y
  void calculate_Hj(int idx, double dx, double dy)
  {

    Hj_ = arma::zeros(2, 9);
    double d = pow(dx, 2) + pow(dy, 2);
    Hj_.at(0, 0) = 0.0;
    Hj_.at(0, 1) = -dx / sqrt(d);
    Hj_.at(0, 2) = -dy / sqrt(d);

    Hj_.at(1, 0) = -1.0;
    Hj_.at(1, 1) = dy / d;
    Hj_.at(1, 2) = -dx / d;


    Hj_.at(0, 3 + (2 * idx)) = dx / sqrt(d);
    Hj_.at(0, 4 + (2 * idx)) = dy / sqrt(d);

    Hj_.at(1, 3 + (2 * idx)) = -dy / d;
    Hj_.at(1, 4 + (2 * idx)) = dx / d;

    // RCLCPP_INFO_STREAM(this->get_logger(), "Hj_: " << Hj_);

  }

  /// @brief Calculate the A matrix
  /// @param dx change in x for predict step
  /// @param dy change in y for predict step
  void calculate_A(double dx, double dy)
  {

    At_ = arma::zeros(9, 9);
    At_.at(1, 0) = -dy;
    At_.at(2, 0) = dx;
    At_ = arma::eye(9, 9) + At_;
  }


  /// @brief Calculate the Q bar matrix
  /// @param q noise value
  void calculate_Q_bar(double q)
  {
    Q_bar_.at(0, 0) = q;
    Q_bar_.at(1, 1) = q;
    Q_bar_.at(2, 2) = q;

  }


  /// @brief Calculate the R matrix
  void calculate_R()
  {
    R_.at(0, 0) = r_;
    R_.at(1, 1) = r_;
  }

  /// @brief Calculate the W matrix
  void calculate_W()
  {
    std::normal_distribution<double> noise_distribution_(0.0, w_);
    double noise = noise_distribution_(generator_);
    for (int i = 0; i < 9; i++) {
      W_.at(i, i) = noise;
    }
  }

  /// @brief Calculate the V matrix
  void calculate_V()
  {
    std::normal_distribution<double> noise_distribution_(0.0, v_);
    double noise = noise_distribution_(generator_);
    for (int i = 0; i < 2; i++) {
      V_.at(i, i) = noise;
    }
  }

  /// @brief Calculate the Sigma bar matrix
  void calculate_Sigma_bar()
  {

    // Calculate W:
    calculate_W();

    Sigma_bar_ = (At_ * Sigma_bar_ * At_.t()) + (W_ * Q_bar_ * W_.t());
    // Sigma_bar_ = (At_ * Sigma_bar_ * At_.t()) + Q_bar_;

  }

  // Helper Functions:
  /// @brief A function to return the yaw from a quaternion
  /// @param q quaternion
  /// @return yaw
  double return_yaw(geometry_msgs::msg::Quaternion q)
  {
    // Begin Citation [7]
    double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    // End Citation [7]
    return yaw;
  }

  /// @brief calculate the z vector
  /// @param idx obstacle index
  /// @return vector z

  arma::vec calculate_z(int idx)
  {

    double range =
      sqrt(
      pow(
        mu_bar_.at(3 + (2 * idx)) - mu_bar_.at(1),
        2) + pow(mu_bar_.at(4 + (2 * idx)) - mu_bar_.at(2), 2));
    double bearing =
      turtlelib::normalize_angle(
      atan2(
        mu_bar_.at(4 + (2 * idx)) - mu_bar_.at(2),
        mu_bar_.at(3 + (2 * idx)) - mu_bar_.at(1)) - mu_bar_.at(0));
    arma::vec z = {range, bearing};

    return z;
  }

  arma::vec cart_to_polar(arma::vec loc)
  {
    double x = loc.at(0);
    double y = loc.at(1);
    double range = sqrt(pow(x, 2) + pow(y, 2));
    double bearing = atan2(y, x);
    arma::vec z = {range, bearing};
    return z;
  }

  /// @brief calculate the z_hat vector
  /// @param z vector z
  /// @return vector z_hat
  arma::vec calculate_z_hat(arma::vec z)
  {
    arma::vec z_hat = arma::zeros(2, 1);
    z_hat.at(0) = sqrt(pow(z.at(0), 2) + pow(z.at(1), 2));
    z_hat.at(1) = turtlelib::normalize_angle(atan2(z.at(1), z.at(0)));
    return z_hat;
  }

  // ROS Helper Functions:
  /// @brief A function to update the green odom message
  void update_green_marker_msg()
  {

    green_marker_array_msg_.markers.clear();
    for (int i = 0; i < int(seen_obs_.size()); i++) {
      visualization_msgs::msg::Marker green_marker;
      green_marker.header.frame_id = "map";
      green_marker.header.stamp = this->now();
      green_marker.ns = "green";
      green_marker.id = seen_obs_.at(i);
      green_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      green_marker.action = visualization_msgs::msg::Marker::ADD;
      green_marker.pose.position.x = mu_.at(3 + (2 * seen_obs_.at(i)));
      green_marker.pose.position.y = mu_.at(4 + (2 * seen_obs_.at(i)));
      green_marker.pose.position.z = 0.0;
      green_marker.pose.orientation.x = 0.0;
      green_marker.pose.orientation.y = 0.0;
      green_marker.pose.orientation.z = 0.0;
      green_marker.pose.orientation.w = 1.0;
      green_marker.scale.x = 0.038 * 2.0;
      green_marker.scale.y = 0.038 * 2.0;
      green_marker.scale.z = 0.25;
      green_marker.color.a = 1.0;
      green_marker.color.r = 0.0;
      green_marker.color.g = 1.0;
      green_marker.color.b = 0.0;
      green_marker_array_msg_.markers.push_back(green_marker);
    }

  }

  /// @brief A function to update the green odom path
  void update_green_odom_path()
  {
    green_odom_path_msg_.header.stamp = this->now();
    green_odom_path_msg_.header.frame_id = odom_id_;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = odom_id_;
    pose.pose.position.x = Tmr_.translation().x;
    pose.pose.position.y = Tmr_.translation().y;
    tf2::Quaternion q;
    q.setRPY(0, 0, Tmr_.rotation());
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    green_odom_path_msg_.poses.push_back(pose);
    green_odom_path_pub_->publish(green_odom_path_msg_);
  }

  /// @brief A function to update the green odom message
  void update_green_odom_msg()
  {
    green_odom_msg_.header.stamp = this->now();
    green_odom_msg_.header.frame_id = odom_id_;
    green_odom_msg_.child_frame_id = body_id_;


  }

  /// @brief A function to set the green odom transform
  void set_green_odom_transform()
  {
    green_odom_tf_.header.stamp = this->now();
    green_odom_tf_.header.frame_id = "green/odom";
    green_odom_tf_.child_frame_id = "green/base_footprint";
    green_odom_tf_.transform.translation.x = prev_green_odom_.pose.pose.position.x;
    green_odom_tf_.transform.translation.y = prev_green_odom_.pose.pose.position.y;
    green_odom_tf_.transform.rotation = prev_green_odom_.pose.pose.orientation;

  }

  /// @brief A function to set the map odom transform
  void set_Tmo_transform()
  {
    map_odom_tf_.header.stamp = this->now();
    map_odom_tf_.header.frame_id = odom_id_;
    map_odom_tf_.child_frame_id = body_id_;
    map_odom_tf_.transform.translation.x = Tmo_.translation().x;
    map_odom_tf_.transform.translation.y = Tmo_.translation().y;
    tf2::Quaternion q;
    q.setRPY(0, 0, Tmo_.rotation());
    map_odom_tf_.transform.rotation.x = q.x();
    map_odom_tf_.transform.rotation.y = q.y();
    map_odom_tf_.transform.rotation.z = q.z();
    map_odom_tf_.transform.rotation.w = q.w();
  }

  /// @brief A function to set the Tmr transform
  void set_Tmr()
  {

    turtlelib::Vector2D r;
    r.x = mu_.at(1);
    r.y = mu_.at(2);
    Tmr_ = turtlelib::Transform2D(r, mu_.at(0));
  }

  /// @brief A function to set the odom to robot transform
  void set_Tor()
  {
    turtlelib::Vector2D o;
    o.x = green_odom_msg_.pose.pose.position.x;
    o.y = green_odom_msg_.pose.pose.position.y;
    double theta = return_yaw(green_odom_msg_.pose.pose.orientation);
    Tor_ = turtlelib::Transform2D(o, theta);

  }

  /// @brief A function to solve for the map to odom transform
  void solve_for_Tmo()
  {

    Tmo_ = Tmr_ * Tor_.inv();

  }

  /// @brief A function to check if an obstacle has been observed
  bool is_observed(int idx)
  {
    for (int i = 0; i < int(seen_obs_.size()); i++) {
      if (seen_obs_.at(i) == idx) {
        return true;
      }
    }
    return false;
  }

  // Initalize Publishers:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr green_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr green_odom_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr green_marker_array_pub_;


  // Initalize Subscribers:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_obstacles_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr reg_obstacles_sub_;
  // Initalize Broadcasters:
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  // Initalize Timers:
  rclcpp::TimerBase::SharedPtr timer_;

  // Initalize the Random Number Generator:
  std::default_random_engine generator_;

  // Initalize Kalman Variables:
  arma::vec mu_bar_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  arma::vec mu_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  arma::mat Sigma_bar_ = arma::eye(9, 9);
  arma::mat Sigma_ = arma::eye(9, 9);

  arma::mat At_ = arma::zeros(9, 9);
  arma::mat Hj_ = arma::zeros(2, 9);

  arma::mat Kt_ = arma::zeros(9, 2);

  arma::mat Q_bar_ = arma::zeros(9, 9);
  arma::mat R_ = arma::zeros(2, 2);

  arma::mat W_ = arma::eye(9, 9);
  arma::mat V_ = arma::eye(2, 2);

  // Initalize ROS Variables:
  nav_msgs::msg::Odometry prev_green_odom_;

  // Initalize Messages:
  nav_msgs::msg::Odometry green_odom_msg_;
  nav_msgs::msg::Path green_odom_path_msg_;
  visualization_msgs::msg::MarkerArray green_marker_array_msg_;
  geometry_msgs::msg::TransformStamped green_odom_tf_;
  geometry_msgs::msg::TransformStamped map_odom_tf_;

  // Seen Obstacles:
  std::vector<int> seen_obs_;

  // Initalize Parameters:
  std::string body_id_;
  std::string odom_id_;
  int num_obs_ = 3;
  double dist_threshold_ = 0.5;

  // Tuning and noise parameters:
  double q_ = 0.01;
  double r_ = 1.0;
  double w_ = 1.0;
  double v_ = 1.0;

  // Define transforms:
  turtlelib::Transform2D Tmr_;
  turtlelib::Transform2D Tor_;
  turtlelib::Transform2D Tmo_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
