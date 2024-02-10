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

TEST_CASE("Test turtle_odom", "[turtle_odom]")
{
  auto node = rclcpp::Node::make_shared("turtle_odom");

  // Create a client to check the if the inital_pose service exists.
  auto ip_client = node->create_client<turtle_control::srv::InitialPose>("initial_pose");
  bool service_found = false;

  // Create a listener to check the odom to base_footprint transform
  auto tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  geometry_msgs::msg::TransformStamped transform;

  rclcpp::Time start_time = rclcpp::Clock().now();

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < 1s)
  )
  {
    // Repeatedly check for the dummy service until its found
    if (ip_client->wait_for_service(0s)) {
      service_found = true;
    }
    try {
      transform = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
    }

    // RCLCPP_INFO(node->get_logger(), "Bool Status %d", service_found);
    rclcpp::spin_some(node);
  }

  // Test assertions - check that the dummy node was found
  CHECK(service_found);
  REQUIRE_THAT(transform.transform.translation.x, Catch::Matchers::WithinAbs(0.0, 0.1));
  REQUIRE_THAT(transform.transform.translation.y, Catch::Matchers::WithinAbs(0.0, 0.1));
  REQUIRE_THAT(transform.transform.translation.z, Catch::Matchers::WithinAbs(0.0, 0.1));
  REQUIRE_THAT(transform.transform.rotation.x, Catch::Matchers::WithinAbs(0.0, 0.1));
  REQUIRE_THAT(transform.transform.rotation.y, Catch::Matchers::WithinAbs(0.0, 0.1));
  REQUIRE_THAT(transform.transform.rotation.z, Catch::Matchers::WithinAbs(0.0, 0.1));
  REQUIRE_THAT(transform.transform.rotation.w, Catch::Matchers::WithinAbs(1.0, 0.1));
}
