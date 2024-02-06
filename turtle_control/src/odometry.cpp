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
#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "turtle_control/srv/initial_pose.hpp"

using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {

    // Declare parameters:
    this->declare_parameter("body_id", " 1");
    body_id_ = this->get_parameter("body_id").as_string();
    if (body_id_ == " ")
    {
        RCLCPP_ERROR(this->get_logger(), "body_id must be a string");
        rclcpp::shutdown();
    }

    this->declare_parameter("odom_id", "2 ");
    odom_id_ = this->get_parameter("odom_id").as_string();
    if (odom_id_ == " ")
    {
        RCLCPP_ERROR(this->get_logger(), "odom_id must be a string");
        rclcpp::shutdown();
    }

    this->declare_parameter("wheel_left", " 3");
    wheel_left_ = this->get_parameter("wheel_left").as_string();
    if (wheel_left_ == " ")
    {
        RCLCPP_ERROR(this->get_logger(), "wheel_left must be a string");
        rclcpp::shutdown();
    }

    this->declare_parameter("wheel_right", " 4");
    wheel_right_ = this->get_parameter("wheel_right").as_string();
    if (wheel_right_ == " ")
    {
        RCLCPP_ERROR(this->get_logger(), "wheel_right must be a string");
        rclcpp::shutdown();
    }

    this->declare_parameter("wheel_radius", -1.0);
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    if (wheel_radius_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "wheel_radius must be greater than 0.0");
        rclcpp::shutdown();
    }

    this->declare_parameter("track_width", -1.0);
    track_width_ = this->get_parameter("track_width").as_double();
    if (track_width_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "track_width must be greater than 0.0");
        rclcpp::shutdown();
    }
    diff_drive_ = turtlelib::DiffDrive(wheel_radius_, track_width_);

    // Define Timers:
    // timer_ = this->create_wall_timer(
    //   std::chrono::duration<double>(1.0 / rate_), std::bind(&Odometry::timer_callback, this));
    

    // Define Publishers:
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

   
    // Define Subscribers:
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&Odometry::joint_state_callback, this, std::placeholders::_1));

    
    // Define Services:
    initial_pose_srv_ = this->create_service<turtle_control::srv::InitialPose>(
      "initial_pose", std::bind(&Odometry::initial_pose_callback, this, std::placeholders::_1, std::placeholders::_2));
    

    // Define Broadcasters:
    odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    
  }

private:
    void initial_pose_callback(
      const std::shared_ptr<turtle_control::srv::InitialPose::Request> request,
      std::shared_ptr<turtle_control::srv::InitialPose::Response>)
    {
      // Send back the current configuration
        RCLCPP_INFO(this->get_logger(), "initial_pose_callback!");
        diff_drive_.set_configuration(request->x, request->y, request->theta);

        turtlelib::Configuration2D config = diff_drive_.get_configuration();

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = body_id_;
        odom_msg.child_frame_id = odom_id_;
        odom_msg.pose.pose.position.x = config.x;
        odom_msg.pose.pose.position.y = config.y;
        odom_msg.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, config.theta);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();


        // Publish the Odometry Message:
        odom_pub_->publish(odom_msg);


        // Send the Transform: 
        transformStamped_.header.stamp = this->get_clock()->now();
        transformStamped_.header.frame_id = body_id_;
        transformStamped_.child_frame_id = odom_id_;
        transformStamped_.transform.translation.x = config.x;
        transformStamped_.transform.translation.y = config.y;
        transformStamped_.transform.translation.z = 0.0;
        transformStamped_.transform.rotation.x = q.x();
        transformStamped_.transform.rotation.y = q.y();
        transformStamped_.transform.rotation.z = q.z();
        transformStamped_.transform.rotation.w = q.w();
        odom_broadcaster_->sendTransform(transformStamped_);
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "joint_state_callback!");
        // Update the Internal Odometry...this is getting wheel positions and doing forward kinematics.
        turtlelib::WheelConfiguration wheel_config;
        wheel_config.theta_l = msg->position[0];
        wheel_config.theta_r = msg->position[1];
        diff_drive_.forward_kinematics(wheel_config);

        turtlelib::Configuration2D config = diff_drive_.get_configuration();

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = body_id_;
        odom_msg.child_frame_id = odom_id_;
        odom_msg.pose.pose.position.x = config.x;
        odom_msg.pose.pose.position.y = config.y;
        odom_msg.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, config.theta);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();


        // Publish the Odometry Message:
        odom_pub_->publish(odom_msg);


        // Send the Transform: 
        transformStamped_.header.stamp = this->get_clock()->now();
        transformStamped_.header.frame_id = body_id_;
        transformStamped_.child_frame_id = odom_id_;
        transformStamped_.transform.translation.x = config.x;
        transformStamped_.transform.translation.y = config.y;
        transformStamped_.transform.translation.z = 0.0;
        transformStamped_.transform.rotation.x = q.x();
        transformStamped_.transform.rotation.y = q.y();
        transformStamped_.transform.rotation.z = q.z();
        transformStamped_.transform.rotation.w = q.w();
        odom_broadcaster_->sendTransform(transformStamped_);

        
    }

  // Initalize Publishers:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;


  // Initialize Subscribers:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
 
  

  // Initialize Services:
  rclcpp::Service<turtle_control::srv::InitialPose>::SharedPtr initial_pose_srv_;
  

  // Initalize Broadcasters:
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
 

  // Initialize Timers:
  rclcpp::TimerBase::SharedPtr timer_;
  


  // Initlaize Messages:
   nav_msgs::msg::Odometry odom_;



  // Initialize Variables:
   double rate_ = 200.0;
  

   turtlelib::DiffDrive diff_drive_;
   geometry_msgs::msg::TransformStamped transformStamped_;

   std::string body_id_;
   std::string odom_id_;
   std::string wheel_left_;
   std::string wheel_right_;

   double track_width_;
   double wheel_radius_;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
