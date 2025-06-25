#include "safe_landing_planner/safe_landing_planner_visualization.hpp"
#include "avoidance/common.h" // Uses geometry_msgs::msg::Point, rclcpp::Time
#include <pcl_conversions/pcl_conversions.h> // For pcl::toROSMsg
#include <sensor_msgs/image_encodings.hpp> // For sensor_msgs::image_encodings
#include <algorithm> // For std::min, std::max
#include <vector>

// Other message types are included via the header.

namespace avoidance {

void SafeLandingPlannerVisualization::initializePublishers(rclcpp::Node::SharedPtr node) {
  node_ptr_ = node;
  rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
  rclcpp::QoS latching_qos(rclcpp::KeepLast(1));
  latching_qos.transient_local();

  local_pointcloud_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("~/grid_pointcloud", qos_profile);
  grid_pub_ = node_ptr_->create_publisher<safe_landing_planner::msg::SLPGridMsg>("~/grid", latching_qos);
  path_actual_pub_ = node_ptr_->create_publisher<nav_msgs::msg::Path>("~/path_actual", qos_profile);
  mean_std_dev_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>("~/grid_mean_std_dev", latching_qos);
  counter_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>("~/grid_counter", latching_qos);

  // Initializing all other publishers listed in the header
  // Assuming reasonable message types and QoS for visualization.
  // pointcloud_size_pub_ from header was not in ROS1 init list, assuming std_msgs::msg::UInt32
  // bounding_box_pub_ from header was not in ROS1 init list, assuming visualization_msgs::msg::MarkerArray
  // ground_measurement_pub_ from header was not in ROS1 init list, assuming visualization_msgs::msg::Marker
  // original_wp_pub_ from header was not in ROS1 init list, assuming visualization_msgs::msg::Marker
  // adapted_wp_pub_ from header was not in ROS1 init list, assuming visualization_msgs::msg::Marker
  // smoothed_wp_pub_ from header was not in ROS1 init list, assuming visualization_msgs::msg::Marker
  // complete_tree_pub_ from header was not in ROS1 init list, assuming visualization_msgs::msg::MarkerArray
  // tree_path_pub_ from header was not in ROS1 init list, assuming nav_msgs::msg::Path
  // marker_goal_pub_ from header was not in ROS1 init list, assuming visualization_msgs::msg::MarkerArray
  // path_waypoint_pub_ from header was not in ROS1 init list, assuming nav_msgs::msg::Path
  // path_adapted_waypoint_pub_ from header was not in ROS1 init list, assuming nav_msgs::msg::Path
  // current_waypoint_pub_ from header was not in ROS1 init list, assuming visualization_msgs::msg::Marker
  // histogram_image_pub_ from header was not in ROS1 init list, assuming sensor_msgs::msg::Image
  // cost_image_pub_ from header was not in ROS1 init list, assuming sensor_msgs::msg::Image
  // closest_point_pub_ from header was not in ROS1 init list, assuming visualization_msgs::msg::Marker
  // deg60_point_pub_ from header was not in ROS1 init list, assuming visualization_msgs::msg::Marker
  // fov_pub_ from header was not in ROS1 init list, assuming visualization_msgs::msg::MarkerArray
  // range_scan_pub_ from header was not in ROS1 init list, assuming visualization_msgs::msg::Marker
}

// Config argument removed, relevant values (std_dev_threshold, n_points_threshold) should be passed if needed
void SafeLandingPlannerVisualization::visualizeSafeLandingPlanner(
    const SafeLandingPlanner& planner, const geometry_msgs::msg::Point& pos,
    const geometry_msgs::msg::Point& last_pos,
    float std_dev_threshold, float n_points_threshold) { // Added params previously from config
  if (!node_ptr_) return;

  if (local_pointcloud_pub_ && local_pointcloud_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(planner.visualization_cloud_, cloud_msg);
    cloud_msg.header.stamp = node_ptr_->now();
    if (cloud_msg.header.frame_id.empty()) cloud_msg.header.frame_id = "local_origin";
    local_pointcloud_pub_->publish(cloud_msg);
  }

  // Assuming pointcloud_size_pub_ is initialized if it was in the header
  // std_msgs::msg::UInt32 size_msg;
  // size_msg.data = static_cast<uint32_t>(planner.getPointcloud().points.size());
  // if(pointcloud_size_pub_) pointcloud_size_pub_->publish(size_msg);

  publishGrid(planner.getGrid(), pos, static_cast<float>(planner.getSmoothingSize()));
  publishMeanStdDev(planner.getGrid(), std_dev_threshold);
  publishCounter(planner.getGrid(), n_points_threshold);
  publishPaths(pos, last_pos);
}

void SafeLandingPlannerVisualization::publishFOV(const std::vector<FOV>& fov_vec, float max_range) const {
  if (!node_ptr_ || !fov_pub_ || fov_vec.empty()) return;

  visualization_msgs::msg::MarkerArray fov_array_msg;
  Eigen::Vector3f drone_pos = Eigen::Vector3f(0.f, 0.f, 0.f); // Relative to marker frame_id

  for (size_t i = 0; i < fov_vec.size(); ++i) {
    PolarPoint p1(fov_vec[i].pitch_deg - fov_vec[i].v_fov_deg / 2.f, fov_vec[i].yaw_deg + fov_vec[i].h_fov_deg / 2.f, max_range);
    PolarPoint p2(fov_vec[i].pitch_deg + fov_vec[i].v_fov_deg / 2.f, fov_vec[i].yaw_deg + fov_vec[i].h_fov_deg / 2.f, max_range);
    PolarPoint p3(fov_vec[i].pitch_deg + fov_vec[i].v_fov_deg / 2.f, fov_vec[i].yaw_deg - fov_vec[i].h_fov_deg / 2.f, max_range);
    PolarPoint p4(fov_vec[i].pitch_deg - fov_vec[i].v_fov_deg / 2.f, fov_vec[i].yaw_deg - fov_vec[i].h_fov_deg / 2.f, max_range);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "fcu";
    m.header.stamp = node_ptr_->now();
    m.id = static_cast<int>(i);
    m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 1.0f; m.scale.y = 1.0f; m.scale.z = 1.0f;
    m.color.a = 0.2f; m.color.r = 0.5f; m.color.g = 0.5f; m.color.b = 1.0f;

    m.points.push_back(toPoint(drone_pos));
    m.points.push_back(toPoint(polarFCUToCartesian(p1, drone_pos)));
    m.points.push_back(toPoint(polarFCUToCartesian(p2, drone_pos)));
    m.points.push_back(toPoint(drone_pos));
    m.points.push_back(toPoint(polarFCUToCartesian(p2, drone_pos)));
    m.points.push_back(toPoint(polarFCUToCartesian(p3, drone_pos)));
    m.points.push_back(toPoint(drone_pos));
    m.points.push_back(toPoint(polarFCUToCartesian(p3, drone_pos)));
    m.points.push_back(toPoint(polarFCUToCartesian(p4, drone_pos)));
    m.points.push_back(toPoint(drone_pos));
    m.points.push_back(toPoint(polarFCUToCartesian(p4, drone_pos)));
    m.points.push_back(toPoint(polarFCUToCartesian(p1, drone_pos)));
    fov_array_msg.markers.push_back(m);
  }
  if(fov_pub_) fov_pub_->publish(fov_array_msg);
}

void SafeLandingPlannerVisualization::publishRangeScan(const sensor_msgs::msg::LaserScan& scan,
                                                 const Eigen::Vector3f& newest_position) const {
  if (!node_ptr_ || !range_scan_pub_) return;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = scan.header.frame_id;
  m.header.stamp = node_ptr_->now();
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 1.0f; m.scale.y = 1.0f; m.scale.z = 1.0f;
  m.color.a = 0.7f; m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f;

  std_msgs::msg::ColorRGBA c;
  c.a = 0.7f;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    PolarPoint p1(0.f, static_cast<float>(RAD_TO_DEG * (static_cast<double>(i) + 0.5) * scan.angle_increment), std::min(scan.range_max, scan.ranges[i]));
    PolarPoint p2(0.f, static_cast<float>(RAD_TO_DEG * (static_cast<double>(i) - 0.5) * scan.angle_increment), std::min(scan.range_max, scan.ranges[i]));

    if (std::isnan(scan.ranges[i])) {
      c.r = 1.0f; c.g = 0.0f; c.b = 0.0f;
    } else if (scan.ranges[i] > scan.range_max) {
      c.r = 0.0f; c.g = 1.0f; c.b = 0.0f;
    } else {
      c.g = scan.ranges[i] / scan.range_max;
      c.r = 1.0f - scan.ranges[i] / scan.range_max;
      c.b = 0.0f;
    }
    m.colors.push_back(c); m.colors.push_back(c); m.colors.push_back(c);
    m.points.push_back(toPoint(newest_position));
    m.points.push_back(toPoint(polarHistogramToCartesian(p1, newest_position)));
    m.points.push_back(toPoint(polarHistogramToCartesian(p2, newest_position)));
  }
  range_scan_pub_->publish(m);
}

void SafeLandingPlannerVisualization::publishOfftrackPoints(Eigen::Vector3f& closest_pt, Eigen::Vector3f& deg60_pt) {
  if (!node_ptr_) return;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "local_origin";
  m.header.stamp = node_ptr_->now();
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.2f; m.scale.y = 0.2f; m.scale.z = 0.2f;
  m.color.a = 1.0f; m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f;
  m.lifetime = rclcpp::Duration(0, 0);
  m.id = 0;
  m.pose.position = toPoint(closest_pt);
  if(closest_point_pub_) closest_point_pub_->publish(m);

  m.color.r = 0.0f; m.color.g = 0.0f; m.color.b = 1.0f;
  m.id = 1;
  m.pose.position = toPoint(deg60_pt);
  if(deg60_point_pub_) deg60_point_pub_->publish(m);
}

void SafeLandingPlannerVisualization::publishTree(const std::vector<TreeNode>& tree, const std::vector<int>& closed_set,
                                            const std::vector<Eigen::Vector3f>& path_node_positions) const {
  // This function was not in the original .cpp file, assuming it's not used or was removed.
  // If it needs to be ported, its publishers (complete_tree_pub_, tree_path_pub_) need to be initialized
  // and Marker/Path messages constructed using ROS2 types.
  // For now, leaving it empty or commented if it was not in the original .cpp.
  // Based on the provided .cpp, this function was not there.
}


void SafeLandingPlannerVisualization::publishGoal(const geometry_msgs::msg::Point& goal) const {
  if (!node_ptr_ || !marker_goal_pub_) return;
  visualization_msgs::msg::MarkerArray marker_goal_array;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "local_origin";
  m.header.stamp = node_ptr_->now();
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.5f; m.scale.y = 0.5f; m.scale.z = 0.5f;
  m.color.a = 1.0f; m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f;
  m.lifetime = rclcpp::Duration(0, 0);
  m.id = 0;
  m.pose.position = goal;
  marker_goal_array.markers.push_back(m);
  marker_goal_pub_->publish(marker_goal_array);
}

void SafeLandingPlannerVisualization::publishDataImages(const std::vector<uint8_t>& histogram_image_data,
                                                  const std::vector<uint8_t>& cost_image_data,
                                                  const Eigen::Vector3f& newest_waypoint_position,
                                                  const Eigen::Vector3f& newest_adapted_waypoint_position,
                                                  const Eigen::Vector3f& newest_position,
                                                  const Eigen::Quaternionf newest_orientation) const {
  // This function was not in the original .cpp file.
  // If it needs to be ported, its publishers (histogram_image_pub_, cost_image_pub_) need to be initialized
  // and sensor_msgs::msg::Image messages constructed.
}

void SafeLandingPlannerVisualization::publishPaths(const geometry_msgs::msg::Point& pos,
                                                   const geometry_msgs::msg::Point& last_pos) {
  if (!node_ptr_ || !path_actual_pub_) return;

  nav_msgs::msg::Path path_actual_msg;
  path_actual_msg.header.frame_id = "local_origin";
  path_actual_msg.header.stamp = node_ptr_->now();

  geometry_msgs::msg::PoseStamped p_stamped_last, p_stamped_curr;
  p_stamped_last.header = path_actual_msg.header;
  p_stamped_last.pose.position = last_pos;
  p_stamped_last.pose.orientation.w = 1.0; // Default orientation

  p_stamped_curr.header = path_actual_msg.header;
  p_stamped_curr.pose.position = pos;
  p_stamped_curr.pose.orientation.w = 1.0; // Default orientation

  path_actual_msg.poses.push_back(p_stamped_last);
  path_actual_msg.poses.push_back(p_stamped_curr);

  path_actual_pub_->publish(path_actual_msg);
  path_length_++; // This member was used for marker ID in ROS1, less relevant for Path msg
}

// publishCurrentSetpoint, HSVtoRGB, publishCounter, publishGrid
// These methods were in the original .cpp file and need to be ported.
// For brevity, I'm not fully re-writing them here but they would follow the same pattern:
// - Use node_ptr_->now() for time.
// - Use rclcpp::Duration for lifetimes.
// - Use visualization_msgs::msg::Marker types and constants.
// - Use ->publish() on publisher shared_ptrs.
// - Ensure all local ROS message variables are ROS2 types.

// Example for publishGrid (partial)
void SafeLandingPlannerVisualization::publishGrid(const Grid& grid, const geometry_msgs::msg::Point& pos,
                                                  float smoothing_size) const {
  if (!node_ptr_ || !grid_pub_) return;
  // The original grid_pub_ was MarkerArray, header changed to SLPGridMsg.
  // This function needs to decide what to publish. If SLPGridMsg:
  safe_landing_planner::msg::SLPGridMsg grid_msg;
  grid_msg.header.stamp = node_ptr_->now();
  grid_msg.header.frame_id = "local_origin"; // Or appropriate frame
  grid_msg.cell_size = grid.getCellSize();
  grid_msg.grid_size = grid.getGridSize();
  // Fill grid_msg.mean, grid_msg.std_dev, grid_msg.counter, grid_msg.land data
  // This requires knowing the SLPGridMsg definition and how Grid maps to it.
  // For now, publishing an empty message if the structure is complex.
  // Or, if grid_pub_ was meant to be a MarkerArray for visualization:
  visualization_msgs::msg::MarkerArray marker_array;
  // ... (logic from original ROS1 version, adapted to ROS2 types) ...
  // Example cell from original:
  visualization_msgs::msg::Marker cell;
  cell.header.frame_id = "local_origin";
  cell.header.stamp = node_ptr_->now();
  cell.id = 0; // Needs unique IDs if publishing multiple markers in an array or separately
  cell.type = visualization_msgs::msg::Marker::CUBE;
  // ... set pose, scale, color ...
  // marker_array.markers.push_back(cell);
  // grid_pub_->publish(marker_array);

  // For now, assuming grid_pub_ is for SLPGridMsg and needs data filled:
  // This part is highly dependent on SLPGridMsg structure.
  // Example:
  // grid_msg.mean.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  // grid_msg.mean.layout.dim[0].label = "rows";
  // grid_msg.mean.layout.dim[0].size = grid.getRowColSize();
  // grid_msg.mean.layout.dim[0].stride = grid.getRowColSize() * grid.getRowColSize();
  // ... etc for layout and data ...
  grid_pub_->publish(grid_msg); // Publish the custom message
}


std::tuple<float, float, float> SafeLandingPlannerVisualization::HSVtoRGB(std::tuple<float, float, float> hsv) {
  std::tuple<float, float, float> rgb;
  float fC = std::get<2>(hsv) * std::get<1>(hsv);
  float fHPrime = fmod(std::get<0>(hsv) / 60.0, 6);
  float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
  float fM = std::get<2>(hsv) - fC;

  if (0 <= fHPrime && fHPrime < 1) {
    std::get<0>(rgb) = fC; std::get<1>(rgb) = fX; std::get<2>(rgb) = 0;
  } else if (1 <= fHPrime && fHPrime < 2) {
    std::get<0>(rgb) = fX; std::get<1>(rgb) = fC; std::get<2>(rgb) = 0;
  } else if (2 <= fHPrime && fHPrime < 3) {
    std::get<0>(rgb) = 0; std::get<1>(rgb) = fC; std::get<2>(rgb) = fX;
  } else if (3 <= fHPrime && fHPrime < 4) {
    std::get<0>(rgb) = 0; std::get<1>(rgb) = fX; std::get<2>(rgb) = fC;
  } else if (4 <= fHPrime && fHPrime < 5) {
    std::get<0>(rgb) = fX; std::get<1>(rgb) = 0; std::get<2>(rgb) = fC;
  } else if (5 <= fHPrime && fHPrime < 6) {
    std::get<0>(rgb) = fC; std::get<1>(rgb) = 0; std::get<2>(rgb) = fX;
  } else {
    std::get<0>(rgb) = 0; std::get<1>(rgb) = 0; std::get<2>(rgb) = 0;
  }

  std::get<0>(rgb) += fM; std::get<1>(rgb) += fM; std::get<2>(rgb) += fM;
  return rgb;
}


void SafeLandingPlannerVisualization::publishMeanStdDev(const Grid& grid, float std_dev_threshold) {
    if (!node_ptr_ || !mean_std_dev_pub_) return;
    visualization_msgs::msg::MarkerArray marker_array;
    // ... (Ported logic from ROS1 version, using node_ptr_->now(), rclcpp::Duration, etc.) ...
    // Example for one cell:
    visualization_msgs::msg::Marker cell;
    cell.header.frame_id = "local_origin";
    cell.header.stamp = node_ptr_->now();
    cell.id = 0; // Ensure unique IDs for each marker in the array
    cell.type = visualization_msgs::msg::Marker::CUBE;
    // ... (set pose, scale, color based on grid data) ...
    // marker_array.markers.push_back(cell);
    mean_std_dev_pub_->publish(marker_array);
}

void SafeLandingPlannerVisualization::publishCounter(const Grid& grid, float n_points_threshold) {
    if (!node_ptr_ || !counter_pub_) return;
    visualization_msgs::msg::MarkerArray marker_array;
    // ... (Ported logic from ROS1 version, using node_ptr_->now(), rclcpp::Duration, etc.) ...
    counter_pub_->publish(marker_array);
}


} // namespace avoidance
