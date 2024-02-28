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
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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
    odom_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("green/odom_path", 10);
    green_marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("green/obstacles", 10);

    // Subscribe to odom topic: 
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&Slam::odom_callback, this, std::placeholders::_1));

    // Subscribe to fake_sensor topic:
    fake_sensor_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/fake_obstacles", 10, std::bind(&Slam::fake_sensor_callback, this, std::placeholders::_1));

    // Define Broadcasters:
    odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Define the timer:
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / 5.0), std::bind(&Slam::time_callback, this));
  }

private:
  void time_callback()
  {
    // Set the transform:
    set_transform();

    // Publish the transform:
    odom_broadcaster_->sendTransform(transformStamped_);

    // Set the odometry message:
    set_odom_msg(mu_(1), mu_(2), mu_(0));

    // Update the odometry path:
    update_odom_path();

    // Publish the odometry path:
    odom_path_pub_->publish(odom_path_);

    // Set the Green Marker Message:
    set_green_marker_msg();

    // Publish the green marker message:
    green_marker_array_pub_->publish(green_marker_array_);

    // Define Map to Robot Transform:
    turtlelib::Vector2D r;
    r.x = mu_.at(1);
    r.y = mu_.at(2);
    turtlelib::Transform2D Tmr_(r, mu_.at(0));

    // Define Odom to Robot Transform:
    turtlelib::Vector2D o;
    o.x = odom_store_.back().pose.pose.position.x;
    o.y = odom_store_.back().pose.pose.position.y;
    turtlelib::Transform2D Tor_(o, odom_store_.back().pose.pose.orientation.z);

    // Solve for the Map to Odom Transform:
    Tmo_ = Tmr_ * Tor_.inv();

  }


  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Store the odometry messages that we receive:
    odom_store_.push_back(*msg);

    // broadcast the transform:
    odom_tf_.header.stamp = this->now();
    odom_tf_.header.frame_id = "green/odom";
    odom_tf_.child_frame_id = "green/base_footprint";
    odom_tf_.transform.translation.x = msg->pose.pose.position.x;
    odom_tf_.transform.translation.y = msg->pose.pose.position.y;
    odom_tf_.transform.translation.z = 0.0;
    odom_tf_.transform.rotation = msg->pose.pose.orientation;
    odom_broadcaster_->sendTransform(odom_tf_);

  }

  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    marker_array_ = *msg;

    // std::vector<geometry_msgs::msg::PoseStamped> obstacle_store;
    geometry_msgs::msg::Pose current_odom = odom_store_.back().pose.pose;
    geometry_msgs::msg::Twist current_twist = odom_store_.back().twist.twist;
    odom_store_.clear();
    
    double odom_dx = current_odom.position.x - prev_odom_.position.x;
    double odom_dy = current_odom.position.y - prev_odom_.position.y;
    double odom_dtheta = current_odom.orientation.z - prev_odom_.orientation.z;

    prev_odom_ = current_odom;

    // Calculate mu_bar_;
    mu_bar_.at(0) = mu_prev_.at(0) + odom_dtheta;
    mu_bar_.at(1) = mu_prev_.at(1) + odom_dx;
    mu_bar_.at(2) = mu_prev_.at(2) + odom_dy;

    // Maybe use the measurement model. 
    mu_bar_.at(3) = mu_prev_.at(3);
    mu_bar_.at(4) = mu_prev_.at(4);
    mu_bar_.at(5) = mu_prev_.at(5);
    mu_bar_.at(6) = mu_prev_.at(6);
    mu_bar_.at(7) = mu_prev_.at(7);
    mu_bar_.at(8) = mu_prev_.at(8);

    // Print the uncorrected state:
    RCLCPP_INFO_STREAM(this->get_logger(), "mu_bar_: " << mu_bar_);

    // Calculate A: 
    // At_ = calculate_A(mu_prev_, current_twist);
    At_ = calculate_A(odom_dx, odom_dy, odom_dtheta);

    // Print A:
    RCLCPP_INFO_STREAM(this->get_logger(), "A: " << At_);

    // Calculate Sigma_Bar:
    Sigma_bar_ = calculate_Sigma_bar(Sigma_prev_, At_);

    // Print Sigma_bar:
    RCLCPP_INFO_STREAM(this->get_logger(), "Sigma_bar: " << Sigma_bar_);

    // This is where we actually implement the SLAM algorithm. 
    arma::vec zi = arma::zeros(6, 1);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Marker Array " << marker_array_.markers.size());
    // RCLCPP_INFO_STREAM(this->get_logger(), "Marker Array " << marker_array_.markers.at(0).pose.position.x);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Marker Array " << marker_array_.markers.at(0).pose.position.y);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Marker Array " << marker_array_.markers.at(1).pose.position.x);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Marker Array " << marker_array_.markers.at(1).pose.position.y);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Marker Array " << marker_array_.markers.at(2).pose.position.x);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Marker Array " << marker_array_.markers.at(2).pose.position.y);

    for (int i = 0; i < int(marker_array_.markers.size()); i++){
      geometry_msgs::msg::Point obstacle_position = marker_array_.markers.at(i).pose.position;
      RCLCPP_INFO_STREAM(this->get_logger(), "Obstacle Position X: " << obstacle_position.x);
      RCLCPP_INFO_STREAM(this->get_logger(), "Obstacle Position Y: " << obstacle_position.y);
      zi.at(i*2) = obstacle_position.x;
      zi.at((i*2) + 1) = obstacle_position.y;
      // zi.push_back(obstacle_position);
    }

    // Print the observation:
    RCLCPP_INFO_STREAM(this->get_logger(), "Observation: " << zi);

    // Calculate Ht_:
    Ht_ = calculate_H(zi);

    // Print Ht_:
    RCLCPP_INFO_STREAM(this->get_logger(), "Ht: " << Ht_);

    // Calculate the Kalman Gain:
    Kt_ = calculate_K(Sigma_bar_, Ht_);

    // Print Kt_:
    RCLCPP_INFO_STREAM(this->get_logger(), "Kt: " << Kt_);

    // Calculate the corrected state:
    mu_ = calculate_mu(mu_bar_, Kt_, Ht_, zi);

    // Print the corrected state:
    RCLCPP_INFO(this->get_logger(), "Corrected State: x: %f, y: %f, theta: %f", mu_(1), mu_(2), mu_(0));

    // Calculate the corrected covariance matrix:
    Sigma_ = calculate_Sigma(Sigma_bar_, Kt_, Ht_);

    // Print the corrected covariance matrix:
    RCLCPP_INFO_STREAM(this->get_logger(), "Sigma: " << Sigma_);

    // Update the previous state:
    mu_prev_ = mu_;

  }

  void set_green_marker_msg()
  {
    // visualization_msgs::msg::MarkerArray green_marker_array;
    green_marker_array_.markers.clear();
    for (int i = 0; i < int(marker_array_.markers.size()); i++){
      visualization_msgs::msg::Marker green_marker;
      green_marker.header.frame_id = "map";
      green_marker.header.stamp = this->now();
      green_marker.ns = "green";
      green_marker.id = i;
      green_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      green_marker.action = visualization_msgs::msg::Marker::ADD;
      green_marker.pose.position.x = mu_.at(3 + (2*i));
      green_marker.pose.position.y = mu_.at(4 + (2*i));
      green_marker.pose.position.z = 0.0;
      green_marker.pose.orientation.x = 0.0;
      green_marker.pose.orientation.y = 0.0;
      green_marker.pose.orientation.z = 0.0;
      green_marker.pose.orientation.w = 1.0;
      green_marker.scale.x = 0.1;
      green_marker.scale.y = 0.1;
      green_marker.scale.z = 0.25;
      green_marker.color.a = 1.0;
      green_marker.color.r = 0.0;
      green_marker.color.g = 1.0;
      green_marker.color.b = 0.0;
      green_marker_array_.markers.push_back(green_marker);
    }
  }

  void update_odom_path()
  {
    // nav_msgs::msg::Path odom_path;
    odom_path_.header.stamp = this->now();
    odom_path_.header.frame_id = odom_id_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = odom_id_;
    pose.pose.position.x = odom_msg_.pose.pose.position.x;
    pose.pose.position.y = odom_msg_.pose.pose.position.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = odom_msg_.pose.pose.orientation.x;
    pose.pose.orientation.y = odom_msg_.pose.pose.orientation.y;
    pose.pose.orientation.z = odom_msg_.pose.pose.orientation.z;
    pose.pose.orientation.w = odom_msg_.pose.pose.orientation.w;
    odom_path_.poses.push_back(pose);
    odom_path_pub_->publish(odom_path_);
  }


  void set_odom_msg(double x, double y, double theta)
  {
    // nav_msgs::msg::Odometry odom_msg;
    odom_msg_.header.stamp = this->now();
    odom_msg_.header.frame_id = odom_id_;
    odom_msg_.child_frame_id = body_id_;
    odom_msg_.pose.pose.position.x = x;
    odom_msg_.pose.pose.position.y = y;
    odom_msg_.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();

    // turtlelib::Twist2D twist = diff_drive_.get_Twist();

    // odom_msg_.twist.twist.linear.x = twist.x;
    // odom_msg_.twist.twist.linear.y = twist.y;
    // odom_msg_.twist.twist.angular.z = twist.omega;
  }

  void set_transform()
  {

    // Get the Map to Odom Transform:
    
    // geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped_.header.stamp = this->get_clock()->now();
    transformStamped_.header.frame_id = odom_id_;
    transformStamped_.child_frame_id = body_id_;
    transformStamped_.transform.translation.x = Tmo_.translation().x;
    transformStamped_.transform.translation.y = Tmo_.translation().y;
    transformStamped_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, Tmo_.rotation());
    transformStamped_.transform.rotation.x = q.x();
    transformStamped_.transform.rotation.y = q.y();
    transformStamped_.transform.rotation.z = q.z();
    transformStamped_.transform.rotation.w = q.w();
  }

  arma::mat calculate_Sigma(arma::mat Sigma_bar, arma::mat Kt, arma::mat Ht){
    arma::mat Sigma = (arma::eye(9, 9) - Kt * Ht) * Sigma_bar;
    return Sigma;
  }

  arma::vec calculate_mu(arma::vec mu_bar, arma::mat Kt, arma::mat Ht, arma::vec zi){
    //TODO:VERIFY THIS FUNCTION
    arma::vec mu = mu_bar + Kt * (arma::vec(zi) - Ht * mu_bar);
    return mu;
  }

  arma::mat calculate_K(arma::mat Sigma_bar, arma::mat Ht){

    arma::mat R = arma::eye(6, 6)*5.0;
    arma::mat Kt = Sigma_bar * Ht.t() * (Ht * Sigma_bar * Ht.t() + R).i();
    return Kt;
  }

  arma::mat calculate_H(arma::vec zi){
    // TODO: Change this function to be dynamics of the number of landmarks we see.
    // TODO: Change this function to not index according to the number of 3 landmarks at a time.
    // TODO: This should only update for a landmark when we see it a get a measurement. 
    arma::mat H = arma::zeros(6, 9);
    RCLCPP_INFO_STREAM(this->get_logger(), "Calculating H");
    RCLCPP_INFO_STREAM(this->get_logger(), "H: " << H);

    for (int i = 0; i < int(zi.n_rows/2); i++){
      RCLCPP_INFO_STREAM(this->get_logger(), "Iteration: " << i);


      double delta_x = mu_bar_.at(3 + i) - mu_bar_.at(1);
      double delta_y = mu_bar_.at((3 + i) + 1) - mu_bar_.at(2);
      // double delta_x = zi(i*2) - mu_bar_(1);
      // double delta_y = zi(i*2 + 1) - mu_bar_(2);
      double d = pow(delta_x, 2) + pow(delta_y, 2);


      RCLCPP_INFO_STREAM(this->get_logger(), "delta_x: " << delta_x);
      RCLCPP_INFO_STREAM(this->get_logger(), "delta_y: " << delta_y);
      RCLCPP_INFO_STREAM(this->get_logger(), "d: " << d);
     
      RCLCPP_INFO_STREAM(this->get_logger(), "Calculating First Sections of H");
      H.at(i*2, 1) = -delta_x / sqrt(d);
      H.at(i*2, 2) = -delta_y / sqrt(d);

      H.at(i*2, (2*i) + 3) = delta_x/sqrt(d);
      H.at(i*2, (2*i) + 4) = delta_y/sqrt(d);

      RCLCPP_INFO_STREAM(this->get_logger(), "Calculating Second Sections of H");
      H.at(i*2 + 1, 0) = -1;
      H.at(i*2 + 1, 1) = delta_y / d;
      H.at(i*2 + 1, 2) = -delta_x / d;

      H.at(i*2 + 1, (2*i) + 3) = -delta_y / d;
      H.at(i*2 + 1, (2*i) + 4) = delta_x / d;
    }
    return H;
  }


 arma::mat calculate_Sigma_bar(arma::mat Sigma_prev_, arma::mat A){
  arma::mat Q = arma::eye(3, 3) *0.01;

  arma::mat Q_bar = arma::zeros(9, 9);
  Q_bar.at(0, 0) = Q.at(0, 0);
  Q_bar.at(1, 1) = Q.at(1, 1);
  Q_bar.at(2, 2) = Q.at(2, 2);


  arma::mat Sigma_bar = (A * Sigma_prev_ * A.t()) + Q_bar;

  return Sigma_bar;
 }

 arma::mat calculate_A(double dx, double dy, double d_theta){

  arma::mat A = arma::zeros(9,9);
  A.at(1,0) = -dy;
  A.at(2,0) = dx;

  A = arma::eye(9,9) + A;

  return A;
 }

//  arma::mat calculate_A(arma::vec mu_prev_, geometry_msgs::msg::Twist current_twist){
    
//     double delta_x = current_twist.linear.x;
//     // double delta_y = current_twist.linear.y;
//     double delta_theta = current_twist.angular.z;
  
//     arma::mat A = arma::zeros(9, 9);
//     arma::mat a = arma::zeros(9, 9);

//     if (turtlelib::almost_equal(delta_theta, 0.0, 1e-9)){
//       a.at(0, 1) = -delta_x*sin(mu_prev_.at(0));
//       a.at(0, 2) = delta_x*cos(mu_prev_.at(0));
      
//     } else {
//       a.at(0, 1) = -(delta_x / delta_theta) * cos(mu_prev_.at(0)) + (delta_x / delta_theta) * cos(mu_prev_.at(0)+ delta_theta);
//       a.at(0, 2) = -(delta_x / delta_theta) * sin(mu_prev_.at(0)) + (delta_x / delta_theta) * sin(mu_prev_.at(0) + delta_theta);

//     }

//     A = arma::eye(9,9) + a;
  
//     return A;

// }

  // Initalize Publishers:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr green_marker_array_pub_;

  // Initialize Subscribers:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;


  // Initalize Broadcasters:
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

  // Initialize Timers:
  rclcpp::TimerBase::SharedPtr timer_;


  // Initialize Variables:
  // double rate_ = 200.0;

  geometry_msgs::msg::TransformStamped transformStamped_;
  geometry_msgs::msg::TransformStamped odom_tf_;
  nav_msgs::msg::Odometry odom_msg_;
  std::vector<nav_msgs::msg::Odometry> odom_store_;
  geometry_msgs::msg::Pose prev_odom_;
  // geometry_msgs::msg::Pose prev_slam_;
  std::vector<geometry_msgs::msg::PoseStamped> obstacle_store_;
  visualization_msgs::msg::MarkerArray marker_array_;
  nav_msgs::msg::Path odom_path_;
  visualization_msgs::msg::MarkerArray green_marker_array_;

  // Initalize State Variables: 
  // arma::vec mu_prev_ = arma::zeros(9,1);
  // arma::vec mu_prev_ = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  arma::vec mu_prev_ = {0.0, 0.0, 0.0, -0.5, -0.7, 0.8, -0.8, 0.4, 0.8};


  // Initalize Current State, uncorrected: 
  // arma::vec mu_bar_ = arma::zeros(9,1);
  // arma::vec mu_bar_ = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  arma::vec mu_bar_ = {0.0, 0.0, 0.0, -0.5, -0.7, 0.8, -0.8, 0.4, 0.8};
  // Initalize Current State, corrected:
  // arma::vec mu_ = arma::zeros(9,1);
  // arma::vec mu_ = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  arma::vec mu_ = {0.0, 0.0, 0.0, -0.5, -0.7, 0.8, -0.8, 0.4, 0.8};


  // Initalize Covariance Matrix:
  arma::mat Sigma_prev_ = arma::eye(9,9);

  // Initalize Covariance Matrix, uncorrected:
  arma::mat Sigma_bar_ = arma::eye(9,9);

  // Initalize Covariance Matrix, corrected:
  arma::mat Sigma_ = arma::eye(9,9);

  // Initalize A matrix:
  arma::mat At_ = arma::zeros(9,9);

  // Initalize Kalman Gain matrix:
  arma::mat Kt_ = arma::eye(9,6);

  // Initalize H matrix:
  arma::mat Ht_ = arma::eye(6,9);

  // Define transforms:
  turtlelib::Transform2D Tmr_;
  turtlelib::Transform2D Tor_;
  turtlelib::Transform2D Tmo_;


  std::string body_id_;
  std::string odom_id_;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}


