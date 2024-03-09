#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>
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
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Landmarks Node...");

    // Declare parameters:

    // Define Publishers:
    cluster_centroid_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/cluster_centroids", 10);

    // Define Subscribers:
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/laser_scan", 10, std::bind(&Landmarks::laser_scan_callback, this, std::placeholders::_1));

    // Define Services:


    // Define the timer:
    // timer_ = this->create_wall_timer(
    //   std::chrono::duration<double>(1.0 / 200.0), std::bind(&Landmarks::time_callback, this));
  }

private:

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Set current lasar scan msg:
    set_laser_scan_msg(msg);

    // Define cluster points:
    define_cluster_points();

    // Filter clusters based on size:
    for (int i = 0; i < int(clusters_.size()); i++)
    {
      if (clusters_.at(i).size() < cluster_size_min_ || clusters_.at(i).size() > cluster_size_max_)
      {
        clusters_.erase(clusters_.begin() + i);
      }
    }

    // Define cluster centroids:
    std::vector<turtlelib::Point2D> centroids;
    for (int i = 0; i < int(clusters_.size()); i++)
    {
      centroids.push_back(calc_centroid(clusters_.at(i)));
    }

    // Set up marker array:
    set_centroid_marker(centroids);

    // Publish Marker at centroid of each cluster:  
    cluster_centroid_pub_->publish(marker_array_);

  }

  // Helper Functions: 
  void set_laser_scan_msg(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    laser_scan_msg_ = msg;
  }

  void set_centroid_marker(std::vector<turtlelib::Point2D> centroids)
  {
    // Clear Marker Array msg:
    marker_array_.markers.clear();
    for (int i = 0; i < int(centroids.size()); i++)
    {
      // Set up marker:
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "red/base_footprint";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "cluster_centroids";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = centroids.at(i).x;
      marker.pose.position.y = centroids.at(i).y;
      marker.pose.position.z = 0.25;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker_array_.markers.push_back(marker);
    }
  }
  
  void define_cluster_points()
  {
    // Initalize the first cluster of points: 
    std::vector<turtlelib::Point2D> cluster;
    cluster.push_back(polar_to_cartesian(laser_scan_msg_->ranges.at(0), 0));

    for (int i = 1; i < int(laser_scan_msg_->ranges.size()); i++)
    {
      // Convert polar to cartesian coordinates in the robot frame:
      turtlelib::Point2D point1 = cluster.back();
      turtlelib::Point2D point2 = polar_to_cartesian(laser_scan_msg_->ranges.at(i), laser_scan_msg_->angle_increment * i);

      // Take point and previous point and calculate distance:
      double distance = calc_distance(point1, point2);

      // If distance is less than threshold, add to cluster:
      if (distance < cluster_distance_threshold_)
      {
        // Add point to cluster:
        cluster.push_back(point2);
      }
      else
      {
        // Start new cluster:
        clusters_.push_back(cluster);
        cluster.clear();
        cluster.push_back(point2);
      }


      // If at the end of the laser scan msg, compare the last point to the first point: 
      if (i == int(laser_scan_msg_->ranges.size()) - 1)
      {
        // Take point and previous point and calculate distance:
        double distance = calc_distance(point2, cluster.at(0));

        // If distance is less than threshold combine the first and last clusters into one:
        if (distance < cluster_distance_threshold_)
        {
          // Create a combine std::vec of the first cluster and the last cluster from clusters_:
          std::vector<turtlelib::Point2D> combined_cluster;
          std::vector<turtlelib::Point2D> first_cluster = clusters_.at(0);
          std::vector<turtlelib::Point2D> last_cluster = cluster;
          combined_cluster.insert(combined_cluster.end(), first_cluster.begin(), first_cluster.end());
          combined_cluster.insert(combined_cluster.end(), last_cluster.begin(), last_cluster.end());
          clusters_.erase(clusters_.begin());
          clusters_.pop_back();
          clusters_.push_back(combined_cluster);



        }
      }
    }
  }

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

  double calc_distance(turtlelib::Point2D point1, turtlelib::Point2D point2)
  {
    // Calculate the distance between two points:
    double distance = sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2));
    return distance;
  }

  turtlelib::Point2D calc_centroid(std::vector<turtlelib::Point2D> cluster)
  {
    // Calculate the centroid of a cluster of points:
    turtlelib::Point2D centroid;
    double x_sum = 0;
    double y_sum = 0;
    for (int i = 0; i < int(cluster.size()); i++)
    {
      x_sum += cluster.at(i).x;
      y_sum += cluster.at(i).y;
    }
    centroid.x = x_sum / cluster.size();
    centroid.y = y_sum / cluster.size();
    return centroid;
  }
  // Initalize Publishers:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_centroid_pub_;


  // Initialize Subscribers:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  // Initialize Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Define msgs to publish:
  visualization_msgs::msg::MarkerArray marker_array_;

  // Initialize Variables:
  sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg_;
  double cluster_distance_threshold_ = 0.1;
  double angle_increment_ = 0.0174533;
  double cluster_size_min_ = 3;
  double cluster_size_max_ = 25;

  // Initalize Cluster tracker:
  std::vector<std::vector<turtlelib::Point2D>> clusters_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
