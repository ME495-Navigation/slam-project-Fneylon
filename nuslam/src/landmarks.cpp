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
    // Define the QoS
    rclcpp::QoS marker_qos(10);
    marker_qos.transient_local();

    // Define Publishers:
    cluster_centroid_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/landmarks", 10);

    // Define Subscribers:
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/laser_scan", 10, std::bind(&Landmarks::laser_scan_callback, this, std::placeholders::_1));
  }

private:

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Set current lasar scan msg:
    set_laser_scan_msg(msg);

    // Define cluster points:
    define_cluster_points();
    // RCLCPP_INFO(this->get_logger(), "Number of Clusters: %d", int(clusters_.size()));

    // Filter clusters based on size:
    clusters_filtered_.clear();
    for (int i = 0; i < int(clusters_.size()); i++) {
      if (clusters_.at(i).size() >= cluster_size_min_ && clusters_.at(i).size() < cluster_size_max_)
      {
        clusters_filtered_.push_back(clusters_.at(i));
      }
    }

    // Define cluster centroids:
    std::vector<turtlelib::Point2D> centroids;
    for (int i = 0; i < int(clusters_filtered_.size()); i++)
    {
      centroids.push_back(calc_centroid(clusters_filtered_.at(i)));
    }

    // Set up marker array:
    // set_centroid_marker(centroids);

    // Publish Marker at centroid of each cluster:  
    // cluster_centroid_pub_->publish(marker_array_);
    int curr_num_clusters = int(centroids.size());

    // Print out the number of clusters:
    if (num_clusters_ != curr_num_clusters)
    {
      // RCLCPP_INFO(this->get_logger(), "Number of Clusters: %d", int(curr_num_clusters));
  
      num_clusters_ = curr_num_clusters;
    }

    // Solve the supervised learning problem of circle regression: 
    std::vector<turtlelib::Point2D> locs;
    std::vector<double> radi;

    // create fake data
    // std::vector<std::vector<turtlelib::Point2D>> temp_clusters;
    // std::vector<turtlelib::Point2D> temp_cluster;
    // arma::vec temp_pt = {0.0, 0.0};
    // turtlelib::Point2D temp_pt;
    // temp_pt.x = 0.0;
    // temp_pt.y = 0.0;


    // // temp_pt = {1.0, 7.0};
    // // turtlelib::Point2D temp_pt;
    // temp_pt.x = 1.0;
    // temp_pt.y = 7.0;
    // temp_cluster.push_back(temp_pt);
    // // temp_pt = {2.0, 6.0};
    // temp_pt.x = 2.0;
    // temp_pt.y = 6.0;
    // temp_cluster.push_back(temp_pt);
    // // temp_pt = {5.0, 8.0};
    // temp_pt.x = 5.0;
    // temp_pt.y = 8.0;
    // temp_cluster.push_back(temp_pt);
    // // temp_pt = {7.0, 7.0};
    // temp_pt.x = 7.0;
    // temp_pt.y = 7.0;
    // temp_cluster.push_back(temp_pt);
    // // temp_pt = {9.0, 5.0};
    // temp_pt.x = 9.0;
    // temp_pt.y = 5.0;
    // temp_cluster.push_back(temp_pt);
    // // temp_pt = {3.0, 7.0};
    // temp_pt.x = 3.0;
    // temp_pt.y = 7.0;
    // temp_cluster.push_back(temp_pt);

    // temp_clusters.push_back(temp_cluster);

    // temp_cluster.clear();

    // // temp_pt = {-1.0, 0.0};
    // temp_pt.x = -1.0;
    // temp_pt.y = 0.0;
    // temp_cluster.push_back(temp_pt);
    // // temp_pt = {-0.3, -0.06};
    // temp_pt.x = -0.3;
    // temp_pt.y = -0.06;
    // temp_cluster.push_back(temp_pt);
    // // temp_pt = {0.3, 0.1};
    // temp_pt.x = 0.3;
    // temp_pt.y = 0.1;
    // temp_cluster.push_back(temp_pt);
    // // temp_pt = {1.0, 0.0};
    // temp_pt.x = 1.0;
    // temp_pt.y = 0.0;
    // temp_cluster.push_back(temp_pt);

    // temp_clusters.push_back(temp_cluster);

    // clusters_filtered_ = temp_clusters;

    for (int i = 0; i < int(clusters_filtered_.size()); i++)
    {
      bool circle_status = circle_regression(clusters_filtered_.at(i));
      if (circle_status) {
        if (turtlelib::almost_equal(curr_radius_, 0.038, 0.001)) {
          locs.push_back(curr_center_);
          radi.push_back(curr_radius_);
        }
        
      }
      // RCLCPP_INFO(this->get_logger(), "Circle Status: %d", circle_status);
    }

    // Set up marker array:
    set_obstacle_marker(locs, radi);

    // Publish the Predicted Marker:
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
  
  void set_obstacle_marker(std::vector<turtlelib::Point2D> locs, std::vector<double> radi){

    // Clear Marker Array msg:
    marker_array_.markers.clear();
    for (int i = 0; i < int(locs.size()); i++)
    {
      // Set up marker:
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "red/base_footprint";
      // marker.header.stamp = this->get_clock()->now();
      marker.ns = "regression_obstacles";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = locs.at(i).x;
      marker.pose.position.y = locs.at(i).y;
      marker.pose.position.z = 0.0;
      marker.scale.x = radi.at(i) * 2.0;
      marker.scale.y = radi.at(i) * 2.0;
      marker.scale.z = 0.25;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker_array_.markers.push_back(marker);
    }
  }
  
  void define_cluster_points()
  {

    // Clear clusters:
    clusters_.clear();

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

  bool circle_regression(std::vector<turtlelib::Point2D> cluster)
  {
    // Solve the supervised learning problem of circle regression:
    // 0. Print number of points in cluster:
    // RCLCPP_INFO(this->get_logger(), "Number of Points in Cluster: %d", int(cluster.size()));

    // 1. Compute the (x,y) coordinates of the centroid of the n data points.
    turtlelib::Point2D centroid = calc_centroid(cluster);

    // 2. Shift the coordinate system so that the centroid is at the origin.
    std::vector<turtlelib::Point2D> shifted_cluster;
    for (int i = 0; i < int(cluster.size()); i++) {
      turtlelib::Point2D shifted_pt;
      shifted_pt.x = cluster.at(i).x - centroid.x;
      shifted_pt.y = cluster.at(i).y - centroid.y;
      shifted_cluster.push_back(shifted_pt);
    }

    // 3. Compute zi = xi^2 + yi^2 for each data point.
    std::vector<double> z;
    for (int i = 0; i < int(shifted_cluster.size()); i++) {
      z.push_back(pow(shifted_cluster.at(i).x, 2) + pow(shifted_cluster.at(i).y, 2));
    }

    // 4. Compute the mean value of z.
    double z_bar = 0.0;
    for (int i = 0; i < int(z.size()); i++){
      z_bar += z.at(i);
    }
    z_bar = z_bar / z.size();

    // 5. Form the matriz Z 
    arma::mat Z;
    for (int i = 0; i < int(shifted_cluster.size()); i++) {
      arma::rowvec row;
      row<< pow(shifted_cluster.at(i).x, 2) + pow(shifted_cluster.at(i).y, 2) << shifted_cluster.at(i).x << shifted_cluster.at(i).y << 1;
      Z.insert_rows(i, row); 
    }

    // 6. Form the matrix M 1/n Z.t * Z
    arma::mat M = (1.0 / shifted_cluster.size()) * Z.t() * Z;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Matrix M: " << M);

    // 7. Form the Constraint Matrix H
    arma::mat H = arma::eye(4,4);
    H.at(0, 0) = 8*z_bar;
    H.at(0, 3) = 2.0;
    H.at(3, 0) = 2.0;
    H.at(3, 3) = 0.0;


    // 8. Compute the inverse of H
    arma::mat H_inv = arma::eye(4,4);
    H_inv.at(3, 0) = 1.0 / 2.0;
    H_inv.at(0, 3) = 1.0 / 2.0;
    H_inv.at(0, 0) = 0.0;
    H_inv.at(3, 3) = -2.0 * z_bar;


    // 9. Compute the SVD of Z 
    arma::mat U;
    arma::vec s;
    arma::mat V;
    svd(U, s, V, Z);

    // Convert the singular values to a diagonal matrix:
    arma::mat Sigma = arma::diagmat(s);

    // 10. If the smallest singular value is less than 10e-12, then let A be the 4th column of the V matrix 
    arma::mat A;

    if (s.at(3) < 10e-12) {
      A = V.col(3);

    // 11. If sigma_4 > 10-12 then let Y = V * Sigma * V.t; Compute the matrix A = Y * A
    } else {

      arma::mat Y = V * Sigma * V.t();
      arma::mat Q = Y * H_inv * Y;

      // Find the eignevalues and eigenvectors of Q
      arma::vec eigval;
      arma::mat eigvec;
      eig_sym(eigval, eigvec, Q);

      // Find the Eigenvector corresponding to the smallest positive eigenvalue of Q: 
      int min_eigval_index = 0;
      double min_eigval = 100000.0;

      for (int i = 0; i < int(eigval.size()); i++) {
        if (eigval.at(i) < min_eigval && eigval.at(i) > 0) {
          min_eigval = eigval.at(i);
          min_eigval_index = i;
        }
      }
      arma::vec A_star = eigvec.col(min_eigval_index);
      A = Y.i() * A_star;
    }

    // RCLCPP_INFO_STREAM(this->get_logger(), "Matrix A: " << A);

    // 12. From equation of a circle compute the center and radius of the circle
    double a = -(A.at(1)) / (2.0 * A.at(0));
    double b = -(A.at(2)) / (2.0 * A.at(0));
    double R_sqr = (pow(A.at(1), 2) + pow(A.at(2), 2) - (4 * A.at(0) * A.at(3))) / (4 * pow(A.at(0), 2));

    // RCLCPP_INFO(this->get_logger(), "Circle Regression: a: %f, b: %f, R_sqr: %f", (a + centroid.x) , (b + centroid.y), sqrt(R_sqr));
    
    // Set the radius variable:
    curr_radius_ = sqrt(R_sqr);
    curr_center_.x = a + centroid.x;
    curr_center_.y = b + centroid.y;

    // 13.  compute the RMSE of the fit and threshold.
    double RMSE = 0.0;
    for (int i = 0; i < int(shifted_cluster.size()); i++) {
      RMSE += pow(pow(shifted_cluster.at(i).x - a, 2) + pow(shifted_cluster.at(i).y - b, 2) - R_sqr, 2);
    }
    RMSE = sqrt(RMSE / shifted_cluster.size());

    // Call Inscribed Angle Theorem to determine if landmark is a circle and avoiding false positives:
    if (RMSE < RMSE_threshold_) {
      // RCLCPP_INFO(this->get_logger(), "Circle Suspected: RMSE: %f", RMSE);
      // if (inscribed_angle_theorem(cluster)) {
      //   RCLCPP_INFO(this->get_logger(), "Circle Confirmed:");
      //   return true;
      // } else {
      //   RCLCPP_INFO(this->get_logger(), "Not a Circle:");
      //   return false;
      // }
    
    } else {
      // RCLCPP_INFO(this->get_logger(), "Not a Circle: RMSE: %f", RMSE);
      return false;
    }
  }

  double calculate_angle(turtlelib::Point2D start, turtlelib::Point2D end, turtlelib::Point2D point)
  {
    // Begin Citation [9]
    // Start = Point A
    // Point = Point B
    // End = Point C
    // Calculate the angle between the start and end points via the inscribed angle theorem:
    // Use the Law of Cosines to calculate the angle:   double a = calc_distance(start, end);
    
    // Calculate the vector between the start and point:
    turtlelib::Vector2D AB;
    AB.x = point.x - start.x;
    AB.y = point.y - start.y;

    // Calculate the vector between the point and end:
    turtlelib::Vector2D BC;
    BC.x = end.x - point.x;
    BC.y = end.y - point.y;

    // Calculate the angle between the two vectors:
    double dot_product = AB.x * BC.x + AB.y * BC.y;
    double magnitude_AB = sqrt(pow(AB.x, 2) + pow(AB.y, 2));
    double magnitude_BC = sqrt(pow(BC.x, 2) + pow(BC.y, 2));
    double angle = acos(dot_product / (magnitude_AB * magnitude_BC));
    // End Citation [9]
    return angle;
  }

  bool inscribed_angle_theorem(std::vector<turtlelib::Point2D> cluster)
  {
    // 1. Set the Start and End Points of the Cluster: 
    turtlelib::Point2D P1 = cluster.at(0);
    turtlelib::Point2D P2 = cluster.at(cluster.size() - 1);

    // 2. For all the points in the cluster, calculate the angle between the start and end points:
    arma::vec inscribed_angles = arma::zeros(cluster.size()-2);
    for (int i = 1; i < int(cluster.size()-1); i++) {
      double angle = calculate_angle(P1, P2, cluster.at(i));
      angle = turtlelib::normalize_angle(angle);
      inscribed_angles.at(i-1) = angle;
    }

    // 3. Calculate the mean of the inscribed angles:
    double mean_angle = arma::mean(inscribed_angles);

    // 4. Calculate the STD of the inscribed angles:
    double std_angle = arma::stddev(inscribed_angles);

    // Determine if the cluster is a circle:
    // If the STD is below 0.15, and the mean between 90 and 135 degrees, then it is circle
    if (std_angle < 0.15 && mean_angle > 1.5708 && mean_angle < 2.35619) {
      return true;
    } else {
      return false;
    }
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
  double cluster_size_min_ = 4;
  double cluster_size_max_ = 25;
  double num_clusters_ = 0;
  double RMSE_threshold_ = 0.25;
  turtlelib::Point2D curr_center_;
  double curr_radius_;

  // Initalize Cluster tracker:
  std::vector<std::vector<turtlelib::Point2D>> clusters_;
  std::vector<std::vector<turtlelib::Point2D>> clusters_filtered_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
