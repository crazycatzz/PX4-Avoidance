#ifndef GLOBAL_PLANNER_BEZIER_H_
#define GLOBAL_PLANNER_BEZIER_H_

#include <math.h>  // sqrt
#include <vector>  // For std::vector
#include <algorithm> // For std::min if used by called functions like distance or middlePoint

#include <nav_msgs/msg/path.hpp> // Updated include
#include <geometry_msgs/msg/point.hpp> // Updated include
#include "global_planner/common.h" // For distance, middlePoint, interpolate

// This file consists functions for functions for Bezier curves

namespace global_planner {

// Returns the point on the quadratic Bezier curve at time t (0 <= t <= 1)
template <typename T>
T quadraticBezier(T p0, T p1, T p2, double t) {
  return ((1 - t) * (1 - t) * p0) + 2 * ((1 - t) * t * p1) + (t * t * p2);
}

// Returns the point on the quadratic Bezier curve, actually independant of t
template <typename T>
T quadraticBezierAcc(T p0, T p1, T p2, double duration = 1.0) {
  return 2 * (p2 - 2 * p1 + p0) / duration * duration;
}

// Returns a quadratic Bezier-curve starting in p0 and and ending in p2
template <typename P>
std::vector<P> threePointBezier(const P& p0, const P& p1, const P& p2, int num_steps = 10) {
  std::vector<P> curve;
  for (int i = 0; i <= num_steps; ++i) {
    double t = ((double)i) / num_steps;
    P new_point;
    new_point.x = quadraticBezier(p0.x, p1.x, p2.x, t);
    new_point.y = quadraticBezier(p0.y, p1.y, p2.y, t);
    new_point.z = quadraticBezier(p0.z, p1.z, p2.z, t);
    curve.push_back(new_point);
  }
  return curve;
}

// Returns a quadratic Bezier-curve starting in p0 and and ending in p2
template <typename PathMessagePtrOrRef> // Generic enough for both Path and Path::SharedPtr etc.
nav_msgs::msg::Path threePointBezier(const PathMessagePtrOrRef& path_input, int num_steps = 10) {
  // Assuming path_input has a 'poses' member like nav_msgs::msg::Path
  // If it's a pointer (like ConstSharedPtr), dereference it.
  // For simplicity, let's assume it's a const nav_msgs::msg::Path& for now.
  // If it can be a shared_ptr, the calling code or this function needs to handle dereferencing.
  // For this specific call from search_tools.h, it's a nav_msgs::msg::Path.
  const nav_msgs::msg::Path& path = path_input; // If PathMessagePtrOrRef is nav_msgs::msg::Path&

  if (path.poses.size() != 3) {
    // Consider using RCLCPP_ERROR or similar if a logger is available, or throw exception
    printf("Path size error for threePointBezier, %zu != 3 \n", path.poses.size());
    return path; // Return original path on error
  }
  nav_msgs::msg::Path new_path = path; // Copy header and other path properties
  auto new_points = threePointBezier(new_path.poses[0].pose.position,
                                     new_path.poses[1].pose.position,
                                     new_path.poses[2].pose.position, num_steps);
  new_path.poses.clear(); // Clear existing poses to fill with smoothed ones
  for (const auto& point : new_points) {
    // Create a new PoseStamped, copying relevant info from an original pose if needed (e.g., orientation)
    // Here, assuming orientation is not changed by smoothing, take from first original pose.
    geometry_msgs::msg::PoseStamped new_pose_stamped = path.poses[0];
    new_pose_stamped.pose.position = point;
    new_path.poses.push_back(new_pose_stamped);
  }
  return new_path;
}

template <typename P, typename BezierMsg>
void fillBezierMsg(BezierMsg& msg, const P& p0, const P& p1, const P& p2, double duration) {
  msg.prev = p0;
  msg.ctrl = p1;
  msg.next = p2;
  msg.duration = duration;
}

// Fills msgs with three BezierMsgs,
// The first message represent accelerating to max_vel
// The second is maintaining max_vel
// The third is decelerating and halting at end
template <typename P, typename BezierMsg>
void bezierFromTwoPoints(const P& start, const P& end, double acc, double max_vel, std::vector<BezierMsg>& msgs) {
  double total_dist = distance(start, end);

  // The duration and distance needed to accerlerate to max_vel
  double acc_duration = max_vel / acc;
  double acc_dist = acc_duration * max_vel / 2;
  // The acceleration phase cannot be more than 50% of the trajectory
  double acc_part = std::min(0.5, acc_dist / total_dist);

  // We accelerate from start to max_vel_point, keep max_vel till decel_point,
  // decelerate to end
  P middle = middlePoint(start, end);
  P max_vel_point = interpolate(start, end, acc_part);
  P decel_point = interpolate(end, start, acc_part);
  double max_vel_duration = distance(max_vel_point, decel_point) / max_vel;

  // Fill the messages
  BezierMsg acc_msg, max_vel_msg, decel_msg;
  fillBezierMsg(acc_msg, start, start, max_vel_point, acc_duration);
  fillBezierMsg(max_vel_msg, max_vel_point, middle, decel_point, max_vel_duration);
  fillBezierMsg(decel_msg, decel_point, end, end, acc_duration);
  msgs = {acc_msg, max_vel_msg, decel_msg};
}

// Puts the control point between start and end such that the acceleration
// and the duration matches the speeds
template <typename P, typename BezierMsg>
void bezierFromTwoSpeeds(const P& start, const P& end, double start_speed, double end_speed, BezierMsg& msg) {
  double avg_speed = (start_speed + end_speed) / 2.0;
  double distance = distance(start, end);
  double duration = distance / avg_speed;
  double c = start_speed / (start_speed + end_speed);
  P ctrl = interpolate(start, end, c);
  fillBezierMsg(msg, start, ctrl, end, duration);
}

// The time it takes to accelerate from p0 to p1, starting with no velocity at
// p0
template <typename P>
double getDuration(const P& p0, const P& p1, double acc) {
  double dist = distance(p0, p1);
  return sqrt(2 * dist / acc);
}

template <typename P>
double getAccelerationMagnitude(const P& p0, const P& p1, const P& p2, double duration) {
  double dx = quadraticBezierAcc(p0.x, p1.x, p2.x);
  double dy = quadraticBezierAcc(p0.y, p1.y, p2.y);
  double dz = quadraticBezierAcc(p0.z, p1.z, p2.z);
  return sqrt(dx * dx + dy * dy + dz * dz) / duration * duration;
}

template <typename BezierMsg>
nav_msgs::msg::Path pathToTriplets(const nav_msgs::msg::Path& path, std::vector<BezierMsg>& triplets, const std::vector<double>& speed) {
  // Note: The original function took `triplets` by value and `speed` by const ref, but didn't seem to use `speed` or return `triplets` effectively.
  // Assuming `triplets` is an out-parameter. And `speed` might be used by `fillBezierMsg` or logic not shown.
  // For now, just porting types and structure.
  if (path.poses.size() < 3) {
    return path;
  }

  nav_msgs::msg::Path result_path = path; // Copy header etc.
  result_path.poses.clear(); // We will fill this if needed, or maybe this func just populates triplets.
                             // The original didn't populate a return path based on triplets.
                             // Let's assume it populates the `triplets` vector.

  // Extract the points from path, duplicate the first and last point to
  // indicate acceleration at the beginning and deceleration at the end
  std::vector<geometry_msgs::msg::Point> points;
  if (!path.poses.empty()) { // Guard against empty path
    points.push_back(path.poses.front().pose.position);
    for (const auto& pose_stamped : path.poses) {
      points.push_back(pose_stamped.pose.position);
    }
    points.push_back(path.poses.back().pose.position);
  } else {
    return path; // Return original empty or invalid path
  }


  triplets.clear(); // Clear out-parameter
  // Original loop condition `i < path.poses.size()` might be off by one due to `points` vector modification.
  // If `points` has N+2 elements (original N poses + duplicated front/back), then path.poses.size() is N.
  // Loop should go up to points[i+1] which means i+1 < points.size(), so i < points.size()-1
  for (size_t i = 1; i < points.size() - 1; i++) { // Iterate through the 'actual' poses in the augmented list
    geometry_msgs::msg::Point prev = middlePoint(points[i - 1], points[i]);
    geometry_msgs::msg::Point ctrl = points[i];
    geometry_msgs::msg::Point next = middlePoint(points[i], points[i + 1]);

    BezierMsg msg;
    // Assuming fillBezierMsg takes geometry_msgs::msg::Point or compatible types for prev, ctrl, next
    // The duration 1.0 is a placeholder from original. Speed might influence this.
    fillBezierMsg(msg, prev, ctrl, next, 1.0);
    triplets.push_back(msg);
  }
  // This function originally returned `path`, not a path made of triplets.
  // If the goal is to return a path visualization of these triplets, that logic is missing.
  // For now, returning the original path as per original, despite generating triplets.
  return path;
}

}  // namespace global_planner
#endif /* GLOBAL_PLANNER_BEZIER_H_ */
