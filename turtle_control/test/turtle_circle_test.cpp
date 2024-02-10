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
#include "turtle_control/srv/control.hpp"
#include "catch_ros2/catch_ros2.hpp"

using namespace std::chrono_literals;
double counter = 0.0;
geometry_msgs::msg::Twist twist;

TEST_CASE("Testing turtle_circle", "[turtle_circle]") {

  auto node = rclcpp::Node::make_shared("turtle_circle");

  auto cmd_vel_subscriber_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](const geometry_msgs::msg::Twist::SharedPtr msg) {
      counter = counter + 1.0;
      twist = *msg;
    });

  auto control_service_client = node->create_client<turtle_control::srv::Control>("control");

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    if (control_service_client->wait_for_service(0s)) {
      break;
    }

    rclcpp::spin_some(node);
  }

  RCLCPP_INFO(node->get_logger(), "Service found");

  // BEGIN CITATION [1]-- MEGAN BLACK //
  //   Initalize the request to be sent to the service;
  auto request = std::make_shared<turtle_control::srv::Control::Request>();
  request->radius = 0.5;
  request->velocity = 0.5;

  // Send the request to the service
  auto result = control_service_client->async_send_request(request);

  // Wait for the result
  while (rclcpp::spin_until_future_complete(node, result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Service called");
  }
// END CITATION [1]-- MEGAN BLACK //

  start_time = rclcpp::Clock().now();
  rclcpp::Time end_time;// = rclcpp::Clock().now();

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    end_time = rclcpp::Clock().now();
    rclcpp::spin_some(node);
  }

  REQUIRE_THAT(
    counter / (end_time - start_time).seconds(),
    Catch::Matchers::WithinAbs(100.0, 10.0));
  // REQUIRE(0 ==1);

}
