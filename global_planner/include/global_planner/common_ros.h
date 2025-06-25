#ifndef GLOBAL_PLANNER_COMMON_ROS_H_
#define GLOBAL_PLANNER_COMMON_ROS_H_

#include <string>
#include <vector> // For std::vector
#include <algorithm> // For std::max, std::abs

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For toMsg, fromMsg, and TF2 transforms
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include <tf2/utils.h> // For tf2::getYaw

#include "global_planner/common.h"  // hasSameYawAndAltitude

// This file contains general functions which have some Ros dependancy

namespace global_planner {

// GLOBAL PLANNER

template <typename P>
tf2::Vector3 toTf2Vector3(const P& point) { // Renamed to avoid conflict if tf::Vector3 is used elsewhere
  return tf2::Vector3(point.x, point.y, point.z);
}

inline double distance(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b) { // Updated type
  return distance(a.pose.position, b.pose.position); // Assumes global_planner::distance works with geometry_msgs::msg::Point
}

inline geometry_msgs::msg::TwistStamped transformTwistMsg(const tf2_ros::Buffer& tf_buffer, // Changed to tf2_ros::Buffer
                                                     const std::string& target_frame,
                                                     // const std::string& fixed_frame, // fixed_frame is part of 'before' header
                                                     const geometry_msgs::msg::TwistStamped& msg) {
  geometry_msgs::msg::TwistStamped transformed_msg = msg; // Copy original message
  geometry_msgs::msg::Vector3Stamped linear_vel_stamped;
  linear_vel_stamped.header = msg.header;
  linear_vel_stamped.vector = msg.twist.linear;

  geometry_msgs::msg::Vector3Stamped angular_vel_stamped;
  angular_vel_stamped.header = msg.header;
  angular_vel_stamped.vector = msg.twist.angular;

  geometry_msgs::msg::Vector3Stamped linear_vel_transformed;
  geometry_msgs::msg::Vector3Stamped angular_vel_transformed;

  try {
    // Transform linear velocity
    tf_buffer.transform(linear_vel_stamped, linear_vel_transformed, target_frame);
    transformed_msg.twist.linear = linear_vel_transformed.vector;

    // Transform angular velocity (frame of angular velocity is tricky, often it's the body frame of the header)
    // If angular velocity is in the frame of msg.header.frame_id, it also needs to be transformed.
    // However, simpler approach for angular velocity is to just copy if it's assumed to be in the new target_frame's orientation already,
    // or if it's an attribute of the body. If it's a vector in space, it needs proper transformation.
    // TF2 transform for Vector3Stamped will rotate it.
    tf_buffer.transform(angular_vel_stamped, angular_vel_transformed, target_frame);
    transformed_msg.twist.angular = angular_vel_transformed.vector;

    transformed_msg.header.frame_id = target_frame; // Update header to new frame
  } catch (const tf2::TransformException &ex) {
    // RCLCPP_WARN or some logger should be used here if available
    // For a header, maybe rethrow or return original? For now, returning original on failure.
    // Consider passing a logger or using a static one.
    // For now, printing to stderr for visibility during porting.
    fprintf(stderr, "Failed to transform twist: %s\n", ex.what());
    return msg; // Return original message if transform fails
  }
  return transformed_msg;
}


// Returns a spectral color between red (0.0) and blue (1.0)
inline std_msgs::msg::ColorRGBA spectralColor(double hue, double alpha = 1.0) { // Updated type
  std_msgs::msg::ColorRGBA color; // Updated type
  color.r = static_cast<float>(std::max(0.0, 2 * hue - 1));
  color.g = static_cast<float>(1.0 - 2.0 * std::abs(hue - 0.5));
  color.b = static_cast<float>(std::max(0.0, 1.0 - 2 * hue));
  color.a = static_cast<float>(alpha);
  return color;
}

template <typename Point, typename Color>
visualization_msgs::msg::Marker createMarker(int id, const Point& position, const Color& color, double scale = 0.1, // Added const&
                                        const std::string& frame_id = "/world") { // Added const&
  visualization_msgs::msg::Marker marker; // Updated type
  marker.id = id;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now(); // Use rclcpp::Clock, consider passing node/clock
  marker.pose.position = position; // Assumes Point is compatible with marker.pose.position (e.g. geometry_msgs::msg::Point)
  marker.type = visualization_msgs::msg::Marker::CUBE; // Updated type
  marker.action = visualization_msgs::msg::Marker::ADD; // Updated type
  marker.scale.x = marker.scale.y = marker.scale.z = static_cast<float>(scale);
  marker.color = color; // Assumes Color is compatible (e.g. std_msgs::msg::ColorRGBA)
  return marker;
}

// Returns true if msg1 and msg2 have both the same altitude and orientation
inline bool hasSameYawAndAltitude(const geometry_msgs::msg::Pose& msg1, const geometry_msgs::msg::Pose& msg2) { // Updated type
  // Compare orientations carefully. Direct equality check for floats is risky.
  // Consider using tf2::getYaw and comparing yaw values with a tolerance.
  // For now, keeping direct comparison as per original logic, but this is a potential issue.
  return std::abs(msg1.orientation.z - msg2.orientation.z) < 1e-4 &&
         std::abs(msg1.orientation.w - msg2.orientation.w) < 1e-4 && // Assuming these components define yaw primarily
         std::abs(msg1.position.z - msg2.position.z) < 1e-4;
}

inline double pathLength(const nav_msgs::msg::Path& path) { // Updated type
  double total_dist = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) { // Use size_t
    total_dist += distance(path.poses[i - 1], path.poses[i]); // Assumes distance works with PoseStamped
  }
  return total_dist;
}

// Returns a path with only the corner points of msg
inline std::vector<geometry_msgs::msg::PoseStamped> filterPathCorners(const std::vector<geometry_msgs::msg::PoseStamped>& msg) { // Updated type
  std::vector<geometry_msgs::msg::PoseStamped> corners; // Initialize empty
  if (msg.empty()) { // Check for empty input
    return corners;
  }

  corners.push_back(msg.front()); // Always add the first point
  if (msg.size() <= 2) { // If 2 or less points, all are corners (or just one if size is 1)
      if (msg.size() == 2 && msg.front().pose.position.x != msg.back().pose.position.x) { // Basic check to avoid duplicate if start=end
          corners.push_back(msg.back());
      } else if (msg.size() == 1) {
          // Only one point, already added
      }
      return corners;
  }


  for (size_t i = 1; i < msg.size() - 1; ++i) { // Use size_t
    // Using geometry_msgs::msg::Point for intermediate calculations
    const geometry_msgs::msg::Point& last = msg[i - 1].pose.position;
    const geometry_msgs::msg::Point& curr = msg[i].pose.position;
    const geometry_msgs::msg::Point& next = msg[i + 1].pose.position;

    // Check for collinearity with a small tolerance
    // Vector from last to curr
    double dx1 = curr.x - last.x;
    double dy1 = curr.y - last.y;
    double dz1 = curr.z - last.z;
    // Vector from curr to next
    double dx2 = next.x - curr.x;
    double dy2 = next.y - curr.y;
    double dz2 = next.z - curr.z;

    // Normalize (or check cross product magnitude)
    // Simplified check: if slopes (or direction vectors) are too similar.
    // This can be made more robust with vector math (e.g. dot product of normalized vectors close to 1 or -1)
    // Or cross product close to zero.
    // For now, keeping original logic structure with a tolerance.
    double tol = 1e-3; // Tolerance for floating point comparisons
    bool same_x_dir = std::abs(dx1 * dy2 - dx2 * dy1) < tol && std::abs(dx1 * dz2 - dx2 * dz1) < tol; // simplified check for 3d
    // The original check was just if the differences are identical, which is too strict for floats.
    // A better check is if the points are collinear.
    // (curr - last) x (next - curr) should be near zero vector if collinear.
    // For simplicity, if direction vector changes significantly, it's a corner.
    // Check if (next - curr) is roughly parallel to (curr - last)
    // A simple way: if (curr - last) and (next - curr) are not linearly dependent (within tolerance)
    // then 'curr' is a corner.
    // The original logic: bool same_x = (next.x - curr.x) == (curr.x - last.x); etc.
    // This implies checking if the segments are part of the same line with same "delta"
    // This is not a general collinearity check.
    // Let's refine: if the *change* in deltas is small, they are on the same line.
    bool is_collinear = (std::abs((next.x - curr.x) - (curr.x - last.x)) < tol &&
                         std::abs((next.y - curr.y) - (curr.y - last.y)) < tol &&
                         std::abs((next.z - curr.z) - (curr.z - last.z)) < tol);


    if (!is_collinear) { // If not collinear, then it's a corner
      corners.push_back(msg[i]);
    }
  }
  if (!msg.empty()) { // Add last point if not already added (e.g. if size was 1 or 2)
      if (corners.empty() || (corners.back().pose.position.x != msg.back().pose.position.x ||
                               corners.back().pose.position.y != msg.back().pose.position.y ||
                               corners.back().pose.position.z != msg.back().pose.position.z )) { // Avoid duplicate
          corners.push_back(msg.back());
      }
  }
  return corners;
}

inline double pathKineticEnergy(const nav_msgs::msg::Path& path) { // Updated type
  if (path.poses.size() < 3) {
    return 0.0;
  }
  std::vector<double> vel_x;
  std::vector<double> vel_y;
  std::vector<double> vel_z;
  for (size_t i = 1; i < path.poses.size(); ++i) { // Use size_t
    vel_x.push_back(path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x);
    vel_y.push_back(path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
    vel_z.push_back(path.poses[i].pose.position.z - path.poses[i - 1].pose.position.z);
  }

  double total_energy = 0.0;
  for (size_t i = 1; i < vel_x.size(); ++i) { // Use size_t
    total_energy += std::abs(vel_x[i] * vel_x[i] - vel_x[i - 1] * vel_x[i - 1]);
    total_energy += std::abs(vel_y[i] * vel_y[i] - vel_y[i - 1] * vel_y[i - 1]);
    total_energy += std::abs(vel_z[i] * vel_z[i] - vel_z[i - 1] * vel_z[i - 1]);
  }
  return total_energy;
}

inline double pathEnergy(const nav_msgs::msg::Path& path, double up_penalty) { // Updated type
  double total_energy = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) { // Use size_t
    total_energy += distance(path.poses[i - 1], path.poses[i]); // Assumes distance works with PoseStamped
    double altitude_increase = path.poses[i].pose.position.z - path.poses[i - 1].pose.position.z;
    total_energy += std::max(0.0, up_penalty * altitude_increase);
  }
  return total_energy;
}

}  // namespace global_planner

#endif /* GLOBAL_PLANNER_COMMON_ROS_H_ */
