#include "avoidance/transform_buffer.h"
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For fromMsg, toMsg
#include <rclcpp/logging.hpp> // For RCLCPP_GET_logger

// Note: rclcpp/clock.hpp, rclcpp/duration.hpp, rclcpp/time.hpp,
// and geometry_msgs/msg/transform_stamped.hpp are included via transform_buffer.h

namespace avoidance {

namespace tf_buffer {

TransformBuffer::TransformBuffer(float buffer_size_s)
    : buffer_size_(rclcpp::Duration::from_seconds(buffer_size_s)) {
  startup_time_ = rclcpp::Clock(RCL_ROS_TIME).now(); // Use RCL_ROS_TIME for simulation compatibility
};

std::string TransformBuffer::getKey(const std::string& source_frame, const std::string& target_frame) const {
  return source_frame + "_to_" + target_frame;
}

bool TransformBuffer::interpolateTransform(const geometry_msgs::msg::TransformStamped& tf_earlier,
                                           const geometry_msgs::msg::TransformStamped& tf_later,
                                           geometry_msgs::msg::TransformStamped& transform) const {
  rclcpp::Time requested_time(transform.header.stamp);
  rclcpp::Time earlier_time(tf_earlier.header.stamp);
  rclcpp::Time later_time(tf_later.header.stamp);

  // check if the requested timestamp lies between the two given transforms
  if (requested_time > later_time || requested_time < earlier_time) {
    return false;
  }

  rclcpp::Duration timeBetween = later_time - earlier_time;
  rclcpp::Duration timeAfterEarlier = requested_time - earlier_time;

  if (timeBetween.nanoseconds() == 0) { // Avoid division by zero if timestamps are identical
      if (timeAfterEarlier.nanoseconds() == 0) { // If requested time is also identical
          transform = tf_earlier; // Or tf_later, they are the same
          return true;
      } else {
          return false; // Cannot interpolate if distinct requested time but identical bounds
      }
  }

  float tau = static_cast<float>(timeAfterEarlier.nanoseconds()) / timeBetween.nanoseconds();

  tf2::Vector3 t_earlier, t_later;
  tf2::fromMsg(tf_earlier.transform.translation, t_earlier);
  tf2::fromMsg(tf_later.transform.translation, t_later);
  tf2::Vector3 translation_tf2 = t_earlier.lerp(t_later, tau); // lerp is equivalent to t_earlier * (1-tau) + t_later * tau
  transform.transform.translation = tf2::toMsg(translation_tf2);

  tf2::Quaternion r_earlier, r_later;
  tf2::fromMsg(tf_earlier.transform.rotation, r_earlier);
  tf2::fromMsg(tf_later.transform.rotation, r_later);
  tf2::Quaternion rotation_tf2 = r_earlier.slerp(r_later, tau);
  transform.transform.rotation = tf2::toMsg(rotation_tf2);

  // frame_id and child_frame_id should be set by the caller or remain from tf_earlier/tf_later
  transform.header.frame_id = tf_earlier.header.frame_id;
  transform.child_frame_id = tf_earlier.child_frame_id;

  return true;
}

bool TransformBuffer::insertTransform(const std::string& source_frame, const std::string& target_frame,
                                      geometry_msgs::msg::TransformStamped transform) {
  std::lock_guard<std::mutex> lck(mutex_);
  std::string key = getKey(source_frame, target_frame);
  auto it = buffer_.find(key);
  if (it == buffer_.end()) {
    buffer_.emplace(key, std::deque<geometry_msgs::msg::TransformStamped>());
    it = buffer_.find(key);
  }

  rclcpp::Time transform_time(transform.header.stamp);
  // check if the given transform is newer than the last buffered one
  if (it->second.empty() || rclcpp::Time(it->second.back().header.stamp) < transform_time) {
    it->second.push_back(transform);
    // remove transforms which are outside the buffer size
    while (!it->second.empty() && (transform_time - rclcpp::Time(it->second.front().header.stamp)) > buffer_size_) {
      it->second.pop_front();
    }
    return true;
  }
  return false;
}

bool TransformBuffer::getTransform(const std::string& source_frame, const std::string& target_frame,
                                   const rclcpp::Time& time, geometry_msgs::msg::TransformStamped& transform) const {
  std::lock_guard<std::mutex> lck(mutex_);
  auto iterator = buffer_.find(getKey(source_frame, target_frame));

  if (iterator == buffer_.end()) {
    print(log_level::error, "TF Buffer: could not retrieve requested transform from buffer, unregistered frames: " + getKey(source_frame, target_frame));
    return false;
  }
  if (iterator->second.empty()) {
    print(log_level::warn, "TF Buffer: could not retrieve requested transform from buffer, buffer is empty for frames: " + getKey(source_frame, target_frame));
    return false;
  }

  rclcpp::Time last_buffered_time(iterator->second.back().header.stamp);
  if (last_buffered_time < time) {
    print(log_level::debug, "TF Buffer: requested transform is newer than latest stored, tf has not yet arrived for frames: " + getKey(source_frame, target_frame));
    return false;
  }

  rclcpp::Time first_buffered_time(iterator->second.front().header.stamp);
  if (first_buffered_time > time) {
    print(log_level::warn, "TF Buffer: requested transform is older than earliest stored, tf has been dropped for frames: " + getKey(source_frame, target_frame));
    return false;
  }

  // Set header for the output transform (stamp will be used by interpolateTransform)
  transform.header.stamp = time; // rclcpp::Time implicitly converts to builtin_interfaces::msg::Time
  transform.header.frame_id = source_frame; // Or target_frame, depending on convention, usually parent
  transform.child_frame_id = target_frame;  // Or source_frame

  // Find the two transforms to interpolate between
  // Iterate from newest to oldest
  const geometry_msgs::msg::TransformStamped* later_tf = nullptr;
  const geometry_msgs::msg::TransformStamped* earlier_tf = nullptr;

  for (auto it_deque = iterator->second.rbegin(); it_deque != iterator->second.rend(); ++it_deque) {
    rclcpp::Time current_tf_time(it_deque->header.stamp);
    if (current_tf_time <= time) {
      earlier_tf = &(*it_deque);
      if (later_tf == nullptr && it_deque != iterator->second.rbegin()) { // Should have a 'later_tf' unless 'time' matches the newest
          auto prev_it_deque = std::prev(it_deque);
          later_tf = &(*prev_it_deque);
      } else if (later_tf == nullptr) { // requested time is older than or equal to the newest, but no tf before it.
          // This case means time matches the newest element, or is between newest and one before it.
          // If time == current_tf_time (newest), we need later_tf to be current_tf_time as well for exact match
          later_tf = earlier_tf; // Use earlier_tf if it's an exact match or only one element
      }
      break;
    }
    later_tf = &(*it_deque);
  }

  // If time matches exactly the newest transform
  if (earlier_tf == nullptr && later_tf != nullptr && rclcpp::Time(later_tf->header.stamp) == time) {
      earlier_tf = later_tf;
  }
  // If time matches exactly the oldest transform
  if (later_tf == nullptr && earlier_tf !=nullptr && rclcpp::Time(earlier_tf->header.stamp) == time) {
      later_tf = earlier_tf;
  }


  if (earlier_tf && later_tf) {
    if (interpolateTransform(*earlier_tf, *later_tf, transform)) {
      return true;
    } else {
      print(log_level::warn, "TF Buffer: could not interpolate transform for frames: " + getKey(source_frame, target_frame));
      return false;
    }
  }

  print(log_level::warn, "TF Buffer: failed to find bounding transforms for interpolation for frames: " + getKey(source_frame, target_frame));
  return false;
}

void TransformBuffer::print(const log_level& level, const std::string& msg) const {
  // Create a logger instance. In a class not derived from Node, get a logger this way.
  auto logger = rclcpp::get_logger("transform_buffer");
  if (rclcpp::Clock(RCL_ROS_TIME).now() - startup_time_ > rclcpp::Duration::from_seconds(3.0)) {
    switch (level) {
      case error: {
        RCLCPP_ERROR(logger, "%s", msg.c_str());
        break;
      }
      case warn: {
        RCLCPP_WARN(logger, "%s", msg.c_str());
        break;
      }
      case info: {
        RCLCPP_INFO(logger, "%s", msg.c_str());
        break;
      }
      case debug: {
        RCLCPP_DEBUG(logger, "%s", msg.c_str());
        break;
      }
    }
  }
}
}
}
