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
    green_marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/green/obstacles", 10);

    // Subscribe to odom topic:
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&Slam::odom_callback, this, std::placeholders::_1));

    // Subscribe to fake_obstacles topic:
    fake_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/fake_obstacles", 10, std::bind(&Slam::fake_obstacles_callback, this, std::placeholders::_1));


    // Define Broadcaster:
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Define the timer:
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / 5.0), std::bind(&Slam::timer_callback, this));

  }

private:

    // ROS Callbacks:
    void timer_callback(){

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

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        
        // Set Green Odom Message:
        green_odom_msg_ = *msg;

        // Calculate the change in x, y, and theta:
        double dx = green_odom_msg_.pose.pose.position.x - prev_green_odom_.pose.pose.position.x;
        double dy = green_odom_msg_.pose.pose.position.y - prev_green_odom_.pose.pose.position.y;
        double dtheta = turtlelib::normalize_angle(return_yaw(green_odom_msg_.pose.pose.orientation) - return_yaw(prev_green_odom_.pose.pose.orientation));

        // Predict the next state:
        predict(dx, dy, dtheta);

        // Calculate the Odom to Robot Transform:
        set_Tor();

        // Set the previous green odom:
        prev_green_odom_ = *msg;
    }

    void fake_obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg){

        auto marker_array = *msg;
        int num_updates = 0;
        
        // Define the for loop to call the update over: 
        for (int i = 0; i < int(marker_array.markers.size()); i++){

            // Extract the id, x, and y from the message:
            int idx = msg->markers.at(i).id;
            double x = msg->markers.at(i).pose.position.x;
            double y = msg->markers.at(i).pose.position.y;

            // Call the update function:
            if (msg->markers.at(i).action == visualization_msgs::msg::Marker::ADD){
                if (is_observed(idx) == true){
                    update(idx, x, y);
                    num_updates++;
                } else {

                    seen_obs_.push_back(idx);
                    mu_bar_.at(3 + (2*idx)) = x + mu_bar_.at(1);
                    mu_bar_.at(4 + (2*idx)) = y + mu_bar_.at(2);
                    mu_.at(3 + (2*idx)) = mu_bar_.at(3 + (2*idx));
                    mu_.at(4 + (2*idx)) = mu_bar_.at(4 + (2*idx));
                }
            }
        }

        if (num_updates == 0){
            mu_ = mu_bar_;
            Sigma_ = Sigma_bar_;
        }
    }

    // Kalman filter functions:
    void predict(double dx, double dy, double dtheta){
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

    void update(int idx, double x, double y){

        // Calculate z_hat: 
        arma::vec z_hat = calculate_z_hat({x, y});

        // Calculate z: 
        arma::vec z = calculate_z(idx);

        // Calculate the difference between the observed and predicted x and y:
        double dx = mu_bar_.at(3 + (2*idx)) - mu_bar_.at(1);
        double dy = mu_bar_.at(4 + (2*idx)) - mu_bar_.at(2);

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
    void update_Sigma(){
        Sigma_ = (arma::eye(9,9) - (Kt_ * Hj_)) * Sigma_bar_;
    }

    void update_mu(arma::vec z, arma::vec z_hat){
        arma::vec z_diff = z_hat - z;
        z_diff.at(1) = turtlelib::normalize_angle(z_diff.at(1));
        mu_ = mu_bar_ + (Kt_ * (z_diff));
    }
    
    void calculate_K(){

        // calculate R:
        calculate_R();

        // calculate V_:
        calculate_V();

        Kt_ = arma::zeros(9,2);
        Kt_ = Sigma_bar_ * Hj_.t() * ((Hj_ * Sigma_bar_ * Hj_.t()) + (V_ * R_ * V_.t())).i();
        // Kt_ = Sigma_bar_ * Hj_.t() * ((Hj_ * Sigma_bar_ * Hj_.t()) + R_).i();

    }
    
    void calculate_Hj(int idx, double dx, double dy){

        Hj_ = arma::zeros(2,9);
        double d = pow(dx, 2) + pow(dy, 2);
        Hj_.at(0, 0) = 0.0;
        Hj_.at(0, 1) = -dx / sqrt(d);
        Hj_.at(0, 2) = -dy / sqrt(d);

        Hj_.at(1, 0) = -1.0;
        Hj_.at(1, 1) = dy / d;
        Hj_.at(1, 2) = -dx / d;


        Hj_.at(0, 3 + (2*idx)) = dx / sqrt(d);
        Hj_.at(0, 4 + (2*idx)) = dy / sqrt(d);

        Hj_.at(1, 3 + (2*idx)) = -dy / d;
        Hj_.at(1, 4 + (2*idx)) = dx / d;

        // RCLCPP_INFO_STREAM(this->get_logger(), "Hj_: " << Hj_);

    }

    void calculate_A(double dx, double dy){
        
        At_ = arma::zeros(9,9);
        At_.at(1,0) = -dy;
        At_.at(2,0) = dx;
        At_ = arma::eye(9,9) + At_;
    }
   
    void calculate_Q_bar(double q){
        Q_bar_.at(0, 0) = q;
        Q_bar_.at(1, 1) = q;
        Q_bar_.at(2, 2) = q;

    }

    void calculate_R(){
        R_.at(0, 0) = r_;
        R_.at(1, 1) = r_;
    }
    
    void calculate_W(){
        std::normal_distribution<double> noise_distribution_(0.0, w_);
        double noise = noise_distribution_(generator_);
        for (int i = 0; i < 9; i++){
            W_.at(i,i) = noise;
        }
    }

    void calculate_V(){
        std::normal_distribution<double> noise_distribution_(0.0, v_);
        double noise = noise_distribution_(generator_);
        for (int i = 0; i < 2; i++){
            V_.at(i,i) = noise;
        }
    }
    
    void calculate_Sigma_bar(){
        
        // Calculate W:
        calculate_W();

        Sigma_bar_ = (At_ * Sigma_bar_ * At_.t()) + (W_ * Q_bar_ * W_.t());
        // Sigma_bar_ = (At_ * Sigma_bar_ * At_.t()) + Q_bar_;

    }

    double return_yaw(geometry_msgs::msg::Quaternion q){
        // Begin Citation [7]
        double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        // End Citation [7]
        return yaw;
    }

    arma::vec calculate_z(int idx){

        double range = sqrt(pow(mu_bar_.at(3 + (2*idx)) - mu_bar_.at(1), 2) + pow(mu_bar_.at(4 + (2*idx)) - mu_bar_.at(2), 2));
        double bearing = turtlelib::normalize_angle(atan2(mu_bar_.at(4 + (2*idx)) - mu_bar_.at(2), mu_bar_.at(3 + (2*idx)) - mu_bar_.at(1)) - mu_bar_.at(0));
        arma::vec z = {range, bearing};

        return z;
    }

    arma::vec calculate_z_hat(arma::vec z){
        arma::vec z_hat = arma::zeros(2,1);
        z_hat.at(0) = sqrt(pow(z.at(0), 2) + pow(z.at(1), 2));
        z_hat.at(1) = turtlelib::normalize_angle(atan2(z.at(1), z.at(0)));
        return z_hat;
    }

    // ROS Helper Functions:
    void update_green_marker_msg(){

        green_marker_array_msg_.markers.clear();
        for (int i = 0; i < int(seen_obs_.size()); i++){
            visualization_msgs::msg::Marker green_marker;
            green_marker.header.frame_id = "map";
            green_marker.header.stamp = this->now();
            green_marker.ns = "green";
            green_marker.id = seen_obs_.at(i);
            green_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            green_marker.action = visualization_msgs::msg::Marker::ADD;
            green_marker.pose.position.x = mu_.at(3 + (2*seen_obs_.at(i)));
            green_marker.pose.position.y = mu_.at(4 + (2*seen_obs_.at(i)));
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
    void update_green_odom_path(){
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
    void update_green_odom_msg(){
        green_odom_msg_.header.stamp = this->now();
        green_odom_msg_.header.frame_id = odom_id_;
        green_odom_msg_.child_frame_id = body_id_;


    }
    void set_green_odom_transform(){
        green_odom_tf_.header.stamp = this->now();
        green_odom_tf_.header.frame_id = "green/odom";
        green_odom_tf_.child_frame_id = "green/base_footprint";
        green_odom_tf_.transform.translation.x = prev_green_odom_.pose.pose.position.x;
        green_odom_tf_.transform.translation.y = prev_green_odom_.pose.pose.position.y;
        green_odom_tf_.transform.rotation = prev_green_odom_.pose.pose.orientation;

    }
    void set_Tmo_transform(){
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
    void set_Tmr(){

        turtlelib::Vector2D r;
        r.x = mu_.at(1);
        r.y = mu_.at(2);
        Tmr_ = turtlelib::Transform2D(r, mu_.at(0));
    }
    void set_Tor(){
        turtlelib::Vector2D o;
        o.x = green_odom_msg_.pose.pose.position.x;
        o.y = green_odom_msg_.pose.pose.position.y;
        double theta = return_yaw(green_odom_msg_.pose.pose.orientation);
        Tor_ = turtlelib::Transform2D(o, theta);

    }
    void solve_for_Tmo(){

        Tmo_ = Tmr_ * Tor_.inv();

    }
    bool is_observed(int idx){
        for (int i = 0; i < int(seen_obs_.size()); i++){
            if (seen_obs_.at(i) == idx){
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
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_obstacles_sub_;

    // Initalize Broadcasters:
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    // Initalize Timers:
    rclcpp::TimerBase::SharedPtr timer_;

    // Initalize the Random Number Generator:
    std::default_random_engine generator_;

    // Initalize Kalman Variables:
    arma::vec mu_bar_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    arma::vec mu_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    arma::mat Sigma_bar_ = arma::eye(9,9);
    arma::mat Sigma_ = arma::eye(9,9);

    arma::mat At_ = arma::zeros(9,9);
    arma::mat Hj_ = arma::zeros(2,9);

    arma::mat Kt_ = arma::zeros(9,2);

    arma::mat Q_bar_ = arma::zeros(9,9);
    arma::mat R_ = arma::zeros(2,2);

    arma::mat W_ = arma::zeros(9,9);
    arma::mat V_ = arma::zeros(2,2);

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

    double q_= 0.01;
    double r_= 0.01;
    double w_ = 0.1;
    double v_ = 0.1;

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


