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
    RCLCPP_INFO(this->get_logger(), "Number of Clusters: %d", int(clusters_.size()));

    // Filter clusters based on size:
    for (int i = 0; i < int(clusters_.size()); i++) {
      if (clusters_.at(i).size() >= cluster_size_min_ && clusters_.at(i).size() < cluster_size_max_)
      {
        clusters_filtered_.push_back(clusters_.at(i));
      }
    }
    // for (int i = 0; i < int(clusters_.size()); i++) {
    //   RCLCPP_INFO(this->get_logger(), "Iteration: %d", i);
    //   RCLCPP_INFO(this->get_logger(), "Cluster Size: %d", int(clusters_.at(i).size()));
    //   if (clusters_.at(i).size() < cluster_size_min_ || clusters_.at(i).size() > cluster_size_max_)
    //   {
    //     RCLCPP_INFO(this->get_logger(), "Erasing Cluster: %d", i);
    //     clusters_.erase(clusters_.begin() + i);
    //   }
    // }

    // RCLCPP_INFO(this->get_logger(), "Number of Clusters: %d", int(clusters_.size()));

    // Define cluster centroids:
    std::vector<turtlelib::Point2D> centroids;
    for (int i = 0; i < int(clusters_filtered_.size()); i++)
    {
      centroids.push_back(calc_centroid(clusters_filtered_.at(i)));
    }

    // Set up marker array:
    set_centroid_marker(centroids);

    // Publish Marker at centroid of each cluster:  
    cluster_centroid_pub_->publish(marker_array_);
    double curr_num_clusters = centroids.size();

    // Print out the number of clusters:
    if (num_clusters_ != curr_num_clusters)
    {
      RCLCPP_INFO(this->get_logger(), "Number of Clusters: %d", int(curr_num_clusters));
      // RCLCPP_INFO_STREAM(this->get_logger(), "Cluster Centroids: " << centroids.at(0).x << ", " << centroids.at(0).y);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Cluster Centroids: " << centroids.at(1).x << ", " << centroids.at(1).y);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Cluster Centroids: " << centroids.at(2).x << ", " << centroids.at(2).y);
      num_clusters_ = curr_num_clusters;
    }

    // Solve the supervised learning problem of circle regression: 
    for (int i = 0; i < int(clusters_filtered_.size()); i++)
    {
      double RMSE = circle_regression(clusters_filtered_.at(i));
      // if (RMSE < RMSE_threshold_)
      {
        // RCLCPP_INFO(this->get_logger(), "Circle Regression RMSE: %f", RMSE);
      }
    }

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


  double circle_regression(std::vector<turtlelib::Point2D> cluster)
  {
    // Solve the supervised learning problem of circle regression:
    // 0. Print number of points in cluster:
    RCLCPP_INFO(this->get_logger(), "Number of Points in Cluster: %d", int(cluster.size()));

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

    // RCLCPP_INFO_STREAM(this->get_logger(), "Matrix Z: " << Z);

    // 6. Form the matrix M 1/n Z.t * Z
    arma::mat M = (1.0 / shifted_cluster.size()) * Z.t() * Z;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Matrix M: " << M);

    // 7. Form the Constraint Matrix H
    arma::mat H = arma::eye(4,4);
    H.at(0, 0) = 8*z_bar;
    H.at(0, 3) = 2.0;
    H.at(3, 0) = 2.0;
    H.at(3, 3) = 0.0;
    // RCLCPP_INFO(this->get_logger(), "Matrix H: %f", H);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Matrix H: " << H);

    // 8. Compute the inverse of H
    arma::mat H_inv = arma::eye(4,4);
    H_inv.at(3, 0) = 1.0 / 2.0;
    H_inv.at(0, 3) = 1.0 / 2.0;
    H_inv.at(0, 0) = 0.0;
    H_inv.at(3, 3) = -2 * z_bar;
    // arma::mat H_inv = H.i();
    // RCLCPP_INFO(this->get_logger(), "Matrix H_inv: %f", H_inv);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Matrix H_inv: " << H_inv);

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
      // RCLCPP_INFO_STREAM(this->get_logger(), "Matrix U: " << U);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Matrix s: " << s);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Matrix V: " << V);
      arma::mat Y = V * Sigma * V.t();
      arma::mat Q = Y * H_inv * Y;

      // RCLCPP_INFO_STREAM(this->get_logger(), "Matrix Q: " << Q);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Matrix Y: " << Y);

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

      // solve Y*A = A_star for A
      A = Y.i() * A_star;
    }

    // RCLCPP_INFO_STREAM(this->get_logger(), "Matrix A: " << A);

    // 12. From equation of a circle compute the center and radius of the circle
    double a = -(A.at(1)) / (2.0 * A.at(0));
    double b = -(A.at(2)) / (2.0 * A.at(0));
    double R_sqr = (pow(A.at(1), 2) + pow(A.at(2), 2) - (4 * A.at(0) * A.at(3))) / (4 * pow(A.at(0), 2));

    RCLCPP_INFO(this->get_logger(), "Circle Regression: a: %f, b: %f, R_sqr: %f", (a + centroid.x) , (b + centroid.y), sqrt(R_sqr));

    // 13.  compute the RMSE of the fit and threshold.
    double RMSE = 0.0;
    for (int i = 0; i < int(shifted_cluster.size()); i++) {
      RMSE += pow(pow(shifted_cluster.at(i).x - a, 2) + pow(shifted_cluster.at(i).y - b, 2) - R_sqr, 2);
    }
    RMSE = sqrt(RMSE / shifted_cluster.size());
    return RMSE;
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
  double RMSE_threshold_ = 0.1;

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
