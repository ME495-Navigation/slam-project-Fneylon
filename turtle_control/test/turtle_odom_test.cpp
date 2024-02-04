#include "catch_ros2/catch_ros2.hpp"
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
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "turtle_control/srv/initial_pose.hpp"

using namespace std::chrono_literals;

TEST_CASE("TurtleOdom", "[turtle_odom]")
{
auto node = rclcpp::Node::make_shared("turtle_odom");

// Create a client to check the if the inital_pose service exists. 
auto ip_client = node->create_client<turtle_control::srv::InitialPose>("test_ip_srv");

// Create a listener to check the odom to base_footprint transform
std::unique_ptr<tf2_ros::Buffer> odom_base_buffer;
std::shared_ptr<tf2_ros::TransformListener> odom_base_listener;

geometry_msgs::msg::TransformStamped transform;

rclcpp::Time start_time = rclcpp::Clock().now();

bool service_found = false;

// rclcpp::Duration time_out = rclcpp::Duration::from_seconds(0);
// rclcpp::Duration duration = rclcpp::Duration::from_seconds(1);

// Keep test running only for the length of the "test_duration" parameter
// (in seconds)
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < 1s)
  )
  {
    // Repeatedly check for the dummy service until its found
    if (ip_client->wait_for_service(0s)) {
      service_found = true;
    }

    // Call the Service

    // Check the transform exists between odom and base_footprint
    transform = odom_base_buffer->lookupTransform("odom", "base_footprint", tf2::TimePointZero);

    rclcpp::spin_some(node);
  }

  // Test assertions - check that the dummy node was found
  CHECK(service_found);

REQUIRE(0 ==1);

}

