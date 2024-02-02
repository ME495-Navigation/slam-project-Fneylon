#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
// #include "turtle_control/turtle_control.hpp"

// using namespace turtle_control::TurtleControl;

TEST_CASE("TurtleControl", "[turtle_control]")
{
  auto node = rclcpp::Node::make_shared("turtle_control");
//   auto client = node->create_client<std_srvs::srv::Empty>("~/stop");
//   auto request = std::make_shared<std_srvs::srv::Empty::Request>();
//   auto result = client->async_send_request(request);
//   REQUIRE(result.get()->success);

    REQUIRE(0 == 1);
}