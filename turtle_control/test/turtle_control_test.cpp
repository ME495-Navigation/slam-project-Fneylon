#include "catch_ros2/catch_ros2.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// #include "turtle_control/turtle_control.hpp"

// using namespace turtle_control::TurtleControl;
nuturtlebot_msgs::msg::WheelCommands wheel_commands;
sensor_msgs::msg::JointState js_msg;

TEST_CASE("Testing turtle_control", "[turtle_control]")
{
  // This starts the node for the tests
  auto node = rclcpp::Node::make_shared("turtle_control");
  double encorder_ticks_per_rad = 651.898646904;
  // node->declare_parameter("motor_cmd_per_rad_sec");
  // node->get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
  
  turtlelib::DiffDrive dd(0.033, 0.16);

  // Create Publishers for the Subscribers 
  auto cmd_vel_test_pub = node->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel",
    10
  );

  auto sensor_test_pub = node->create_publisher<nuturtlebot_msgs::msg::SensorData>(
    "sensor",
    10
  );

  // Create Subscribers for the Publishers
  auto wheel_cmd_test_pub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd",
    10, [](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) 
    
    {
      wheel_commands = *msg;
    });

  auto joint_state_test_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    10, [](const sensor_msgs::msg::JointState::SharedPtr msg) 
    {
      js_msg = *msg;
    });

  rclcpp::Time start_time = rclcpp::Clock().now();

   // Test case for verifying the cmd_vel commands with pure translation results in correct wheel commands
    geometry_msgs::msg::Twist trans_twist;
    trans_twist.linear.x = 1.0;
    trans_twist.linear.y = 0.0;
    trans_twist.angular.z = 0.0;

   
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    cmd_vel_test_pub->publish(trans_twist);
    // cmd_vel_test_pub->publish(rot_twist);
    rclcpp::spin_some(node);
    
  }

  turtlelib::Twist2D twist;
  twist.omega = trans_twist.angular.z;
  twist.x = trans_twist.linear.x;
  twist.y = trans_twist.linear.y;

  turtlelib::WheelConfiguration wheel_vel = dd.inverse_kinematics(twist);
  // RCLCPP_INFO(node->get_logger(), "Trans_Twist: %f, %f, %f", trans_twist.linear.x, trans_twist.linear.y, trans_twist.angular.z);
  // RCLCPP_INFO(node->get_logger(), "Twist: %f, %f, %f", twist.x, twist.y, twist.omega);
  // RCLCPP_INFO(node->get_logger(), "Wheel Velocities: %f, %f", wheel_vel.theta_l, wheel_vel.theta_r);
  // RCLCPP_INFO(node->get_logger(), "Wheel Commands: %d, %d", wheel_commands.left_velocity, wheel_commands.right_velocity);

  REQUIRE_THAT(wheel_commands.left_velocity, Catch::Matchers::WithinAbs(int(wheel_vel.theta_l), 1.0e-12));
  REQUIRE_THAT(wheel_commands.right_velocity, Catch::Matchers::WithinAbs(int(wheel_vel.theta_r), 1.0e-12));

  rclcpp::Time start_time2 = rclcpp::Clock().now();
   // Test case for verifying the cmd_vel commands with pure rotation results in correct wheel commands
    geometry_msgs::msg::Twist rot_twist;
    rot_twist.linear.x = 0.0;
    rot_twist.linear.y = 0.0;
    rot_twist.angular.z = 1.0;

    nuturtlebot_msgs::msg::SensorData sensor_data;
    sensor_data.left_encoder = 100;
    sensor_data.right_encoder = 100;
    
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time2) < rclcpp::Duration::from_seconds(2))
  )
  {
    cmd_vel_test_pub->publish(rot_twist);
    sensor_test_pub->publish(sensor_data);
    rclcpp::spin_some(node);
    
  }

  // Test Case for verifying the cmd_vel commands with pure rotation results in correct wheel commands
  turtlelib::Twist2D twist2;
  twist2.omega = rot_twist.angular.z;
  twist2.x = rot_twist.linear.x;
  twist2.y = rot_twist.linear.y;

  turtlelib::WheelConfiguration wheel_vel2 = dd.inverse_kinematics(twist2);
  REQUIRE_THAT(wheel_commands.left_velocity, Catch::Matchers::WithinAbs(int(wheel_vel2.theta_l), 1.0e-12));
  REQUIRE_THAT(wheel_commands.right_velocity, Catch::Matchers::WithinAbs(int(wheel_vel2.theta_r), 1.0e-12));

  // Test Case for verifying the encorder data on sensors is converted to joint states properly
  // Get the joint states and convert that to the sensor data.
  // nuturtlebot_msgs::msg::SensorData sensor_data2;
  // sensor_data2.left_encoder = 10; //int(joint_states.position[0] * encorder_ticks_per_rad);
  // sensor_data2.right_encoder = int(joint_states.position[1] * encorder_ticks_per_rad);
  // REQUIRE_THAT(sensor_data2.left_encoder, Catch::Matchers::WithinAbs(sensor_data.left_encoder, 1.0e-12));
  // REQUIRE_THAT(sensor_data2.right_encoder, Catch::Matchers::WithinAbs(sensor_data.right_encoder, 1.0e-12));

  // REQUIRE_THAT(js_msg.position[0], Catch::Matchers::WithinAbs(int(sensor_data.left_encoder/encorder_ticks_per_rad), 1.0e-12));
  // REQUIRE_THAT(js_msg.position[1], Catch::Matchers::WithinAbs(int(sensor_data.right_encoder/encorder_ticks_per_rad), 1.0e-12));
  // RCLCPP_INFO_STREAM(node->get_logger(), "joint_states: "<<js_msg);
  RCLCPP_INFO_STREAM(node->get_logger(), "joint_states: "<<js_msg.position[0]);
  RCLCPP_INFO_STREAM(node->get_logger(), "joint_states: "<<js_msg.position[1]);

  REQUIRE_THAT(js_msg.position.at(0), Catch::Matchers::WithinAbs(int(sensor_data.left_encoder/encorder_ticks_per_rad), 1.0e-12));
  REQUIRE_THAT(js_msg.position.at(1), Catch::Matchers::WithinAbs(int(sensor_data.right_encoder/encorder_ticks_per_rad), 1.0e-12));

  // REQUIRE(joint_states.position[0] == 0);

  // RCLCPP_INFO(node->get_logger(), "joint_states: %f, %f", js_msg.position[0], js_msg.position[1]);





  // REQUIRE(0 == 1);
}


