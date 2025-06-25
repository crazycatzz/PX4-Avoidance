#include "local_planner/local_planner_visualization.h"

#include "avoidance/common.h" // Uses geometry_msgs::msg::Point, rclcpp::Time
#include "local_planner/planner_functions.h" // Uses rclcpp::Time
#include "local_planner/tree_node.h"

#include <sensor_msgs/msg/image.hpp> // For sensor_msgs::msg::Image
#include <pcl_conversions/pcl_conversions.h> // For pcl::toROSMsg

// visualization_msgs are included via local_planner_visualization.h

namespace avoidance {

// initialize publishers for local planner visualization topics
void LocalPlannerVisualization::initializePublishers(rclcpp::Node::SharedPtr node) {
  node_ptr_ = node; // Store the node pointer
  rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
  rclcpp::QoS latching_qos(rclcpp::KeepLast(1));
  latching_qos.transient_local();


  local_pointcloud_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("~/local_pointcloud", qos_profile);
  pointcloud_size_pub_ = node_ptr_->create_publisher<std_msgs::msg::UInt32>("~/pointcloud_size", qos_profile);
  bounding_box_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>("~/bounding_box", latching_qos);
  ground_measurement_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::Marker>("~/ground_measurement", latching_qos);
  original_wp_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::Marker>("~/original_waypoint", latching_qos);
  adapted_wp_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::Marker>("~/adapted_waypoint", latching_qos);
  smoothed_wp_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::Marker>("~/smoothed_waypoint", latching_qos);
  complete_tree_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>("~/complete_tree", latching_qos); // Changed to MarkerArray
  tree_path_pub_ = node_ptr_->create_publisher<nav_msgs::msg::Path>("~/tree_path", latching_qos); // Changed to Path
  marker_goal_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>("~/goal_position", latching_qos);
  path_actual_pub_ = node_ptr_->create_publisher<nav_msgs::msg::Path>("~/path_actual", qos_profile); // Changed to Path
  path_waypoint_pub_ = node_ptr_->create_publisher<nav_msgs::msg::Path>("~/path_waypoint", qos_profile); // Changed to Path
  path_adapted_waypoint_pub_ = node_ptr_->create_publisher<nav_msgs::msg::Path>("~/path_adapted_waypoint", qos_profile); // Changed to Path
  current_waypoint_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::Marker>("~/current_setpoint", latching_qos);
  histogram_image_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Image>("~/histogram_image", latching_qos);
  cost_image_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Image>("~/cost_image", latching_qos);
  closest_point_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::Marker>("~/closest_point", latching_qos);
  deg60_point_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::Marker>("~/deg60_point", latching_qos);
  fov_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>("~/fov", latching_qos); // Changed to MarkerArray
  range_scan_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::Marker>("~/range_scan", latching_qos);
}

void LocalPlannerVisualization::visualizePlannerData(const LocalPlanner& planner,
                                                     const Eigen::Vector3f& newest_waypoint_position,
                                                     const Eigen::Vector3f& newest_adapted_waypoint_position,
                                                     const Eigen::Vector3f& newest_position,
                                                     const Eigen::Quaternionf& newest_orientation) const {
  if (!node_ptr_) return; // Guard if node_ptr_ isn't set

  // visualize clouds
  sensor_msgs::msg::PointCloud2 cloud_msg;
  // planner.getPointcloud() returns const pcl::PointCloud<pcl::PointXYZI>&
  // It might not have a ROS header. We need to set it.
  pcl::toROSMsg(planner.getPointcloud(), cloud_msg);
  cloud_msg.header.stamp = node_ptr_->now();
  cloud_msg.header.frame_id = "local_origin"; // Or planner.frame_id_ if available
  local_pointcloud_pub_->publish(cloud_msg);

  std_msgs::msg::UInt32 size_msg; // Updated type
  size_msg.data = static_cast<uint32_t>(planner.getPointcloud().points.size()); // Use .points.size() for PCL cloud
  pointcloud_size_pub_->publish(size_msg);

  // visualize tree calculation
  std::vector<TreeNode> tree;
  std::vector<int> closed_set;
  std::vector<Eigen::Vector3f> path_node_positions;
  planner.getTree(tree, closed_set, path_node_positions);
  publishTree(tree, closed_set, path_node_positions);

  // visualize goal
  publishGoal(toPoint(planner.getGoal()));

  // publish histogram image
  publishDataImages(planner.histogram_image_data_, planner.cost_image_data_, newest_waypoint_position,
                    newest_adapted_waypoint_position, newest_position, newest_orientation);

  // publish the FOV
  publishFOV(planner.getFOV(), planner.getSensorRange());

  // range scan
  publishRangeScan(planner.distance_data_, newest_position);
}

void LocalPlannerVisualization::publishFOV(const std::vector<FOV>& fov_vec, float max_range) const {
  Eigen::Vector3f drone_pos = Eigen::Vector3f(0.f, 0.f, 0.f);
  for (int i = 0; i < fov_vec.size(); ++i) {
    PolarPoint p1(fov_vec[i].pitch_deg - fov_vec[i].v_fov_deg / 2.f, fov_vec[i].yaw_deg + fov_vec[i].h_fov_deg / 2.f,
                  max_range);
    PolarPoint p2(fov_vec[i].pitch_deg + fov_vec[i].v_fov_deg / 2.f, fov_vec[i].yaw_deg + fov_vec[i].h_fov_deg / 2.f,
                  max_range);
    PolarPoint p3(fov_vec[i].pitch_deg + fov_vec[i].v_fov_deg / 2.f, fov_vec[i].yaw_deg - fov_vec[i].h_fov_deg / 2.f,
                  max_range);
    PolarPoint p4(fov_vec[i].pitch_deg - fov_vec[i].v_fov_deg / 2.f, fov_vec[i].yaw_deg - fov_vec[i].h_fov_deg / 2.f,
                  max_range);

    visualization_msgs::msg::Marker m; // Updated type
    m.header.frame_id = "fcu"; // This should be consistent with TF tree, ideally from a parameter or planner state
    m.header.stamp = node_ptr_->now(); // Updated time
    m.id = i;
    m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST; // Updated type
    m.action = visualization_msgs::msg::Marker::ADD; // Updated type
    m.scale.x = 1.0f;
    m.scale.y = 1.0f;
    m.scale.z = 1.0f;
    m.color.a = 0.4f;
    m.color.r = 0.5f;
    m.color.g = 0.5f;
    m.color.b = 1.0f;

    // side 1
    m.points.push_back(toPoint(drone_pos));
    m.points.push_back(toPoint(polarFCUToCartesian(p1, drone_pos)));
    m.points.push_back(toPoint(polarFCUToCartesian(p2, drone_pos)));
    // side 2
    m.points.push_back(toPoint(drone_pos));
    m.points.push_back(toPoint(polarFCUToCartesian(p2, drone_pos)));
    m.points.push_back(toPoint(polarFCUToCartesian(p3, drone_pos)));
    // side 3
    m.points.push_back(toPoint(drone_pos));
    m.points.push_back(toPoint(polarFCUToCartesian(p3, drone_pos)));
    m.points.push_back(toPoint(polarFCUToCartesian(p4, drone_pos)));
    // side 4
    m.points.push_back(toPoint(drone_pos));
    m.points.push_back(toPoint(polarFCUToCartesian(p1, drone_pos))); // toPoint returns geometry_msgs::msg::Point
    m.points.push_back(toPoint(polarFCUToCartesian(p1, drone_pos)));

    // fov_pub_ expects MarkerArray, this publishes Marker. Needs fix if fov_pub_ type is correct in header.
    // For now, assuming fov_pub_ in header was Marker and this is a single marker per FOV item.
    // If fov_pub_ is MarkerArray, then:
    // visualization_msgs::msg::MarkerArray fov_array_msg;
    // fov_array_msg.markers.push_back(m);
    // fov_pub_->publish(fov_array_msg);
    // For now, assuming fov_pub_ was meant to be Marker for this loop, or this function needs to build an array.
    // Based on header change to MarkerArray for fov_pub_:
    // This function should accumulate markers and publish once or publish one by one if IDs are unique.
    // For simplicity, let's assume it's one marker per call for now if IDs are handled.
    // However, the header defines fov_pub_ as MarkerArray, so this should be:
    // visualization_msgs::msg::MarkerArray fov_array; ... fov_array.markers.push_back(m); ... fov_pub_->publish(fov_array);
    // This function is const, so it cannot modify a member MarkerArray to accumulate.
    // It should either take MarkerArray as arg, or publish one marker at a time with unique IDs.
    // The original `fov_pub_ = nh.advertise<visualization_msgs::Marker>("fov", 4);` suggests it was one marker.
    // But the updated header has MarkerArray. I will assume it should publish one marker with ID `i`.
    if (fov_pub_ && fov_pub_->get_topic_name()) { // Check if publisher is valid
      visualization_msgs::msg::MarkerArray temp_array; temp_array.markers.push_back(m); // Temporary fix for type mismatch if any
      fov_pub_->publish(temp_array); // This will publish an array with one marker
    }
  }
}

void LocalPlannerVisualization::publishRangeScan(const sensor_msgs::msg::LaserScan& scan, // Type from header
                                                 const Eigen::Vector3f& newest_position) const {
  if (!node_ptr_) return;
  visualization_msgs::msg::Marker m; // Updated type
  m.header.frame_id = "local_origin"; // Should be a parameter or from scan.header.frame_id if appropriate
  m.header.stamp = node_ptr_->now(); // Updated time
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST; // Updated type
  m.action = visualization_msgs::msg::Marker::ADD; // Updated type
  m.scale.x = 1.0f;
  m.scale.y = 1.0f;
  m.scale.z = 1.0f;
  // Default color, will be overridden per point
  m.color.a = 0.7f;
  m.color.r = 1.0f;
  m.color.g = 1.0f;
  m.color.b = 1.0f;

  std_msgs::msg::ColorRGBA c; // Updated type
  c.a = 0.7f;

  for (size_t i = 0; i < scan.ranges.size(); ++i) { // Use size_t
    // Ensure PolarPoint constructor and other functions handle float arguments
    PolarPoint p1(0.f, static_cast<float>(RAD_TO_DEG * (static_cast<double>(i) + 0.5) * scan.angle_increment), std::min(scan.range_max, scan.ranges[i]));
    PolarPoint p2(0.f, static_cast<float>(RAD_TO_DEG * (static_cast<double>(i) - 0.5) * scan.angle_increment), std::min(scan.range_max, scan.ranges[i]));

    if (std::isnan(scan.ranges[i])) {
      c.r = 1.0;
      c.g = 0.0;
      c.b = 0.0;
    } else if (scan.ranges[i] > scan.range_max) {
      c.r = 0.0;
      c.g = 1.0;
      c.b = 0.0;

    } else {
      c.g = scan.ranges[i] / scan.range_max;
      c.r = 1.0 - scan.ranges[i] / scan.range_max;
      c.b = 0.0;
    }
    m.colors.push_back(c);
    m.colors.push_back(c);
    m.colors.push_back(c);

    // side 1
    m.points.push_back(toPoint(newest_position)); // toPoint returns geometry_msgs::msg::Point
    m.points.push_back(toPoint(polarHistogramToCartesian(p1, newest_position)));
    m.points.push_back(toPoint(polarHistogramToCartesian(p2, newest_position)));
  }
  if(range_scan_pub_) range_scan_pub_->publish(m);
}

void LocalPlannerVisualization::publishOfftrackPoints(Eigen::Vector3f& closest_pt, Eigen::Vector3f& deg60_pt) {
  if (!node_ptr_) return;
  visualization_msgs::msg::Marker m; // Updated type

  m.header.frame_id = "local_origin"; // Should be a parameter
  m.header.stamp = node_ptr_->now(); // Updated time
  m.type = visualization_msgs::msg::Marker::SPHERE; // Updated type
  m.action = visualization_msgs::msg::Marker::ADD; // Updated type
  m.scale.x = 0.2f;
  m.scale.y = 0.2f;
  m.scale.z = 0.2f;
  m.color.a = 1.0f;
  m.color.r = 1.0f;
  m.color.g = 0.0f;
  m.color.b = 0.0f;
  m.lifetime = rclcpp::Duration(0, 0); // Infinite
  m.id = 0;
  m.pose.position.x = closest_pt.x();
  m.pose.position.y = closest_pt.y();
  m.pose.position.z = closest_pt.z();
  if(closest_point_pub_) closest_point_pub_->publish(m);

  m.color.r = 0.0f;
  m.color.g = 0.0f;
  m.color.b = 1.0f;
  m.id = 1; // Different ID
  m.pose.position.x = deg60_pt.x();
  m.pose.position.y = deg60_pt.y();
  m.pose.position.z = deg60_pt.z();
  if(deg60_point_pub_) deg60_point_pub_->publish(m);
}

void LocalPlannerVisualization::publishTree(const std::vector<TreeNode>& tree, const std::vector<int>& closed_set,
                                            const std::vector<Eigen::Vector3f>& path_node_positions) const {
  if (!node_ptr_) return;
  visualization_msgs::msg::MarkerArray tree_marker_array; // Publish as MarkerArray if complete_tree_pub_ is MarkerArray
  visualization_msgs::msg::Marker tree_marker_segment; // For individual segments if needed, or build up points in one marker

  tree_marker_segment.header.frame_id = "local_origin"; // Should be a parameter
  tree_marker_segment.header.stamp = node_ptr_->now(); // Updated time
  tree_marker_segment.id = 0;
  tree_marker_segment.type = visualization_msgs::msg::Marker::LINE_LIST; // Updated type
  tree_marker_segment.action = visualization_msgs::msg::Marker::ADD; // Updated type
  tree_marker_segment.pose.orientation.w = 1.0;
  tree_marker_segment.scale.x = 0.05;
  tree_marker_segment.color.a = 0.8f;
  tree_marker_segment.color.r = 0.4f;
  tree_marker_segment.color.g = 0.0f;
  tree_marker_segment.color.b = 0.6f;

  nav_msgs::msg::Path path_marker_msg; // tree_path_pub_ is nav_msgs::Path
  path_marker_msg.header.frame_id = "local_origin"; // Should be a parameter
  path_marker_msg.header.stamp = node_ptr_->now(); // Updated time

  tree_marker_segment.points.reserve(closed_set.size() * 2);
  for (size_t i = 0; i < closed_set.size(); i++) {
    int node_nr = closed_set[i];
    if (node_nr < tree.size() && tree[node_nr].origin_ < tree.size() && tree[node_nr].origin_ != node_nr) { // Basic sanity check
        geometry_msgs::msg::Point p1 = toPoint(tree[node_nr].getPosition());
        int origin_idx = tree[node_nr].origin_;
        geometry_msgs::msg::Point p2 = toPoint(tree[origin_idx].getPosition());
        tree_marker_segment.points.push_back(p1);
        tree_marker_segment.points.push_back(p2);
    }
  }
  tree_marker_array.markers.push_back(tree_marker_segment);


  path_marker_msg.poses.reserve(path_node_positions.size());
  for (size_t i = 0; i < path_node_positions.size(); i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_marker_msg.header; // Inherit stamp and frame_id
    pose.pose.position = toPoint(path_node_positions[i]);
    pose.pose.orientation.w = 1.0; // Default orientation
    path_marker_msg.poses.push_back(pose);
  }

  if(complete_tree_pub_) complete_tree_pub_->publish(tree_marker_array); // Publishes array
  if(tree_path_pub_) tree_path_pub_->publish(path_marker_msg);
}

void LocalPlannerVisualization::publishGoal(const geometry_msgs::msg::Point& goal) const { // Type from header
  if (!node_ptr_) return;
  visualization_msgs::msg::MarkerArray marker_goal_array; // Publisher is MarkerArray
  visualization_msgs::msg::Marker m; // Updated type

  m.header.frame_id = "local_origin"; // Should be a parameter
  m.header.stamp = node_ptr_->now(); // Updated time
  m.type = visualization_msgs::msg::Marker::SPHERE; // Updated type
  m.action = visualization_msgs::msg::Marker::ADD; // Updated type
  m.scale.x = 0.5f;
  m.scale.y = 0.5f;
  m.scale.z = 0.5f;
  m.color.a = 1.0f;
  m.color.r = 1.0f;
  m.color.g = 1.0f;
  m.color.b = 0.0f;
  m.lifetime = rclcpp::Duration(0, 0); // Infinite
  m.id = 0;
  m.pose.position = goal; // goal is already geometry_msgs::msg::Point
  marker_goal_array.markers.push_back(m);
  if(marker_goal_pub_) marker_goal_pub_->publish(marker_goal_array);
}

void LocalPlannerVisualization::publishDataImages(const std::vector<uint8_t>& histogram_image_data,
                                                  const std::vector<uint8_t>& cost_image_data,
                                                  const Eigen::Vector3f& newest_waypoint_position,
                                                  const Eigen::Vector3f& newest_adapted_waypoint_position,
                                                  const Eigen::Vector3f& newest_position,
                                                  const Eigen::Quaternionf newest_orientation) const {
  if (!node_ptr_) return;
  sensor_msgs::msg::Image cost_img; // Updated type
  cost_img.header.stamp = node_ptr_->now(); // Updated time
  cost_img.header.frame_id = "local_origin"; // Or a more appropriate frame
  cost_img.height = GRID_LENGTH_E;
  cost_img.width = GRID_LENGTH_Z;
  cost_img.encoding = "rgb8"; // sensor_msgs::image_encodings::RGB8;
  cost_img.is_bigendian = 0; // Should be false for most systems
  cost_img.step = 3 * cost_img.width;
  cost_img.data = cost_image_data;

  // current orientation
  float curr_yaw_fcu_frame = getYawFromQuaternion(newest_orientation);
  float yaw_angle_histogram_frame = -static_cast<float>(curr_yaw_fcu_frame) * 180.0f / M_PI_F + 90.0f;
  PolarPoint heading_pol(0, yaw_angle_histogram_frame, 1.0);
  Eigen::Vector2i heading_index = polarToHistogramIndex(heading_pol, ALPHA_RES);

  // current setpoint
  PolarPoint waypoint_pol = cartesianToPolarHistogram(newest_waypoint_position, newest_position);
  Eigen::Vector2i waypoint_index = polarToHistogramIndex(waypoint_pol, ALPHA_RES);
  PolarPoint adapted_waypoint_pol = cartesianToPolarHistogram(newest_adapted_waypoint_position, newest_position);
  Eigen::Vector2i adapted_waypoint_index = polarToHistogramIndex(adapted_waypoint_pol, ALPHA_RES);

  // color in the image
  if (cost_img.data.size() == static_cast<size_t>(3 * GRID_LENGTH_E * GRID_LENGTH_Z)) { // Use static_cast for comparison
    // current heading blue
    cost_img.data[colorImageIndex(heading_index.y(), heading_index.x(), 2)] = 255; // uint8_t

    // waypoint white
    cost_img.data[colorImageIndex(waypoint_index.y(), waypoint_index.x(), 0)] = 255;
    cost_img.data[colorImageIndex(waypoint_index.y(), waypoint_index.x(), 1)] = 255;
    cost_img.data[colorImageIndex(waypoint_index.y(), waypoint_index.x(), 2)] = 255;

    // adapted waypoint light blue
    cost_img.data[colorImageIndex(adapted_waypoint_index.y(), adapted_waypoint_index.x(), 1)] = 255;
    cost_img.data[colorImageIndex(adapted_waypoint_index.y(), adapted_waypoint_index.x(), 2)] = 255;
  }

  // histogram image
  sensor_msgs::msg::Image hist_img; // Updated type
  hist_img.header.stamp = node_ptr_->now(); // Updated time
  hist_img.header.frame_id = "local_origin"; // Or a more appropriate frame
  hist_img.height = GRID_LENGTH_E;
  hist_img.width = GRID_LENGTH_Z;
  hist_img.encoding = sensor_msgs::image_encodings::MONO8; // This namespace is usually available
  hist_img.is_bigendian = 0; // Should be false
  hist_img.step = hist_img.width; // For MONO8, step is width
  hist_img.data = histogram_image_data;

  if(histogram_image_pub_) histogram_image_pub_->publish(hist_img);
  if(cost_image_pub_) cost_image_pub_->publish(cost_img);
}

void LocalPlannerVisualization::visualizeWaypoints(const Eigen::Vector3f& goto_position,
                                                   const Eigen::Vector3f& adapted_goto_position,
                                                   const Eigen::Vector3f& smoothed_goto_position) const {
  if (!node_ptr_) return;
  visualization_msgs::msg::Marker sphere1, sphere2, sphere3; // Updated type

  rclcpp::Time now = node_ptr_->now(); // Updated time

  sphere1.header.frame_id = "local_origin"; // Should be a parameter
  sphere1.header.stamp = now;
  sphere1.id = 0;
  sphere1.type = visualization_msgs::msg::Marker::SPHERE; // Updated type
  sphere1.action = visualization_msgs::msg::Marker::ADD; // Updated type
  sphere1.pose.position = toPoint(goto_position); // toPoint returns geometry_msgs::msg::Point
  sphere1.pose.orientation.x = 0.0;
  sphere1.pose.orientation.y = 0.0;
  sphere1.pose.orientation.z = 0.0;
  sphere1.pose.orientation.w = 1.0;
  sphere1.scale.x = 0.2f;
  sphere1.scale.y = 0.2f;
  sphere1.scale.z = 0.2f;
  sphere1.color.a = 0.8f;
  sphere1.color.r = 0.5f;
  sphere1.color.g = 1.0f;
  sphere1.color.b = 0.0f;

  sphere2 = sphere1; // Copy common properties
  sphere2.id = 1; // Different ID
  sphere2.pose.position = toPoint(adapted_goto_position);
  sphere2.color.r = 1.0f;
  sphere2.color.g = 1.0f;
  sphere2.color.b = 0.0f;

  sphere3 = sphere1; // Copy common properties
  sphere3.id = 2; // Different ID
  sphere3.pose.position = toPoint(smoothed_goto_position);
  sphere3.color.r = 1.0f;
  sphere3.color.g = 0.5f;
  sphere3.color.b = 0.0f;

  if(original_wp_pub_) original_wp_pub_->publish(sphere1);
  if(adapted_wp_pub_) adapted_wp_pub_->publish(sphere2);
  if(smoothed_wp_pub_) smoothed_wp_pub_->publish(sphere3);
}

void LocalPlannerVisualization::publishPaths(const Eigen::Vector3f& last_position,
                                             const Eigen::Vector3f& newest_position, const Eigen::Vector3f& last_wp,
                                             const Eigen::Vector3f& newest_wp, const Eigen::Vector3f& last_adapted_wp,
                                             const Eigen::Vector3f& newest_adapted_wp) {
  if (!node_ptr_) return;
  rclcpp::Time now = node_ptr_->now();

  // publish actual path (as nav_msgs::Path)
  nav_msgs::msg::Path path_actual_msg; // Updated type
  path_actual_msg.header.frame_id = "local_origin"; // Should be a parameter
  path_actual_msg.header.stamp = now;
  geometry_msgs::msg::PoseStamped p_last, p_new;
  p_last.header = path_actual_msg.header;
  p_new.header = path_actual_msg.header;
  p_last.pose.position = toPoint(last_position); p_last.pose.orientation.w = 1.0;
  p_new.pose.position = toPoint(newest_position); p_new.pose.orientation.w = 1.0;
  path_actual_msg.poses.push_back(p_last);
  path_actual_msg.poses.push_back(p_new);
  if(path_actual_pub_) path_actual_pub_->publish(path_actual_msg);

  // publish path set by calculated waypoints (as nav_msgs::Path)
  nav_msgs::msg::Path path_waypoint_msg; // Updated type
  path_waypoint_msg.header = path_actual_msg.header; // Copy header
  p_last.pose.position = toPoint(last_wp);
  p_new.pose.position = toPoint(newest_wp);
  path_waypoint_msg.poses.push_back(p_last);
  path_waypoint_msg.poses.push_back(p_new);
  if(path_waypoint_pub_) path_waypoint_pub_->publish(path_waypoint_msg);

  // publish path set by adapted waypoints (as nav_msgs::Path)
  nav_msgs::msg::Path path_adapted_waypoint_msg; // Updated type
  path_adapted_waypoint_msg.header = path_actual_msg.header; // Copy header
  p_last.pose.position = toPoint(last_adapted_wp);
  p_new.pose.position = toPoint(newest_adapted_wp);
  path_adapted_waypoint_msg.poses.push_back(p_last);
  path_adapted_waypoint_msg.poses.push_back(p_new);
  if(path_adapted_waypoint_pub_) path_adapted_waypoint_pub_->publish(path_adapted_waypoint_msg);

  // path_length_++; // This member was not used for ID in ROS1 for paths, markers yes.
}

void LocalPlannerVisualization::publishCurrentSetpoint(const geometry_msgs::msg::Twist& wp, // Type from header
                                                       const PlannerState& waypoint_type,
                                                       const Eigen::Vector3f& newest_position) const {
  if (!node_ptr_) return;
  visualization_msgs::msg::Marker setpoint; // Updated type
  setpoint.header.frame_id = "local_origin"; // Should be a parameter
  setpoint.header.stamp = node_ptr_->now(); // Updated time
  setpoint.id = 0;
  setpoint.type = visualization_msgs::msg::Marker::ARROW; // Updated type
  setpoint.action = visualization_msgs::msg::Marker::ADD; // Updated type

  geometry_msgs::msg::Point tip;    // Updated type
  geometry_msgs::msg::Point newest_pos_msg; // Updated type

  newest_pos_msg.x = newest_position(0);
  newest_pos.y = newest_position(1);
  newest_pos.z = newest_position(2);
  tip.x = newest_pos.x + wp.linear.x;
  tip.y = newest_pos.y + wp.linear.y;
  tip.z = newest_pos.z + wp.linear.z;
  setpoint.points.push_back(newest_pos);
  setpoint.points.push_back(tip);
  setpoint.scale.x = 0.1;
  setpoint.scale.y = 0.1;
  setpoint.scale.z = 0.1;
  setpoint.color.a = 1.0;

  switch (waypoint_type) {
    case PlannerState::LOITER: {
      setpoint.color.r = 1.0;
      setpoint.color.g = 1.0;
      setpoint.color.b = 0.0;
      break;
    }
    case PlannerState::TRY_PATH: {
      setpoint.color.r = 0.0;
      setpoint.color.g = 1.0;
      setpoint.color.b = 0.0;
      break;
    }
    case PlannerState::DIRECT: {
      setpoint.color.r = 0.0;
      setpoint.color.g = 0.0;
      setpoint.color.b = 1.0;
      break;
    }
    case PlannerState::ALTITUDE_CHANGE: {
      setpoint.color.r = 1.0;
      setpoint.color.g = 0.0;
      setpoint.color.b = 1.0;
      break;
    }
  }

  current_waypoint_pub_.publish(setpoint);
}
}
