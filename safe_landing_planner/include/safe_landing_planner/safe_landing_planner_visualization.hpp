#pragma once

#include <geometry_msgs/msg/point.hpp> // Updated
#include <rclcpp/rclcpp.hpp>          // For Node::SharedPtr and publisher types
#include "safe_landing_planner.hpp"   // Assumes this will be/is updated

// Additional message types for publishers
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <safe_landing_planner/msg/slp_grid_msg.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string> // For frame_id in createMarker if used
#include <vector>
#include <tuple> // For HSVtoRGB

namespace avoidance {

class SafeLandingPlannerVisualization {
 public:
  SafeLandingPlannerVisualization() = default;
  ~SafeLandingPlannerVisualization() = default;

  /**
  * @brief      initializes all publishers used for local planner visualization
  **/
  void initializePublishers(rclcpp::Node::SharedPtr node); // Changed to take rclcpp::Node::SharedPtr

  /**
  * @brief injects into the SafeLandingPlannerVisualization class, all the data
  *to be visualized
  * @param[in] planner, SafeLandingPlanner class
  * @param[in] pos, current vehicle position
  * @param[in] last_pos, previous vehicle position
  * @param[in] config, dynamic reconfigure parameters
  **/
  // Removed config from signature, parameters should be obtained from the node or passed individually if needed for viz
  void visualizeSafeLandingPlanner(const SafeLandingPlanner& planner, const geometry_msgs::msg::Point& pos, // Updated type
                                   const geometry_msgs::msg::Point& last_pos); // Updated type

 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pointcloud_pub_;
  rclcpp::Publisher<safe_landing_planner::msg::SLPGridMsg>::SharedPtr grid_pub_; // Custom grid message
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_actual_pub_; // Changed to Path
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mean_std_dev_pub_; // Assuming MarkerArray for grid viz
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr counter_pub_;      // Assuming MarkerArray for grid viz

  rclcpp::Node::SharedPtr node_ptr_; // To store the node handle for creating publishers and getting time

  int path_length_ = 0;

  /**
  * @brief       Visualization of the actual path of the drone and the path of
  *the waypoint
  * @params[in]  pos, location of the drone at the last timestep
  * @params[in]  last_pos, location of the drone at the previous timestep
  **/
  void publishPaths(const geometry_msgs::msg::Point& pos, const geometry_msgs::msg::Point& last_pos); // Updated type

  /**
  * @brief       Visualization of the boolean land grid
  * @params[in]  grid, grid data structure
  * @params[in]  pos, vehicle position
  * @param[in]   smoothing_size, kernel size on cell land hysteresis
  **/
  void publishGrid(const Grid& grid, const geometry_msgs::msg::Point& pos, float smoothing_size) const; // Updated type

  /**
  * @brief      Visualization of the mean values grid
  * @params[in] grid, grid data structure
  * @params[in] std_dev_threshold, maximum standard deviation cell value to land
  **/
  void publishMeanStdDev(const Grid& grid, float std_dev_threshold);

  /**
  * @brief      Visualization of the standard deviation values grid
  * @params[in]  grid, grid data structure
  * @params[in]  n_points_threshold, minimum number of points per cell parameter
  **/
  void publishCounter(const Grid& grid, float n_points_threshold);

  /**
  * @brief converts from HSV to RGB color space
  * @param[in] hsv, Hue in range [0, 360], Saturation and Value in range [0, 1]
  * @return[out] rgb, Red Blue Green, each elemnt in range [0, 1]
  */
  std::tuple<float, float, float> HSVtoRGB(std::tuple<float, float, float> hsv);
};
}
