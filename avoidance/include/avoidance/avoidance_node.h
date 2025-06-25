#ifndef AVOIDANCE_AVOIDANCE_NODE_H
#define AVOIDANCE_AVOIDANCE_NODE_H

#include <rclcpp/rclcpp.hpp> // Replaces ros/ros.h and ros/callback_queue.h

#include <mavros_msgs/msg/param.hpp>           // Updated include
#include <mavros_msgs/srv/param_get.hpp>       // Updated include for service
#include <mavros_msgs/msg/waypoint_list.hpp>   // Updated include
#include "avoidance/common.h"                   // Already updated common.h
#include <mavros_msgs/msg/companion_process_status.hpp> // Updated include

#include <thread>
#include <mutex> // For std::mutex with param_cb_mutex_

namespace avoidance {

class AvoidanceNode : public rclcpp::Node { // Inherit from rclcpp::Node
 public:
  explicit AvoidanceNode(const rclcpp::NodeOptions & options); // Updated constructor
  ~AvoidanceNode();
  /**
  * @brief      check healthiness of the avoidance system to trigger failsafe in
  *             the FCU
  * @param[in]  since_last_cloud, time elapsed since the last waypoint was
  *             published to the FCU
  * @param[in]  since_start, time elapsed since staring the node
  * @param[out] planner_is_healthy, true if the planner is running without
  *errors
  * @param[out] hover, true if the vehicle is hovering
  **/
  void checkFailsafe(rclcpp::Duration since_last_cloud, rclcpp::Duration since_start, bool& hover); // Updated rclcpp::Duration

  ModelParameters getPX4Parameters() const;
  float getMissionItemSpeed() const { return mission_item_speed_; }
  MAV_STATE getSystemStatus();

  /**
  * @brief     polls PX4 Firmware paramters every 30 seconds
  **/
  void checkPx4Parameters();

  void setSystemStatus(MAV_STATE state);
  void init(); // Implementation will change significantly

  /**
  * @brief     callaback with the list of FCU Mission Items
  * @param[in] msg, list of mission items
  **/
  void missionCallback(const mavros_msgs::msg::WaypointList::ConstSharedPtr msg); // Updated signature

 private:
  // Removed nh_ and nh_private_

  rclcpp::Publisher<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr mavros_system_status_pub_;
  rclcpp::Subscription<mavros_msgs::msg::Param>::SharedPtr px4_param_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr mission_sub_;
  rclcpp::Client<mavros_msgs::srv::ParamGet>::SharedPtr get_px4_param_client_;

  rclcpp::TimerBase::SharedPtr cmdloop_timer_, statusloop_timer_;
  // Removed CallbackQueues and AsyncSpinners, use ROS2 executors

  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_STANDBY;

  ModelParameters px4_;  // PX4 Firmware paramters
  std::unique_ptr<std::mutex> param_cb_mutex_; // Keep as is

  std::thread worker_; // Keep as is, though its management might change

  double cmdloop_dt_, statusloop_dt_;
  double timeout_termination_;
  double timeout_critical_;
  double timeout_startup_;

  bool position_received_;
  bool should_exit_;

  float mission_item_speed_;

  void cmdLoopCallback(); // Updated signature
  void statusLoopCallback(); // Updated signature
  void publishSystemStatus();

  /**
  * @brief     callaback with the list of FCU parameters
  * @param[in] msg, list of paramters
  **/
  void px4ParamsCallback(const mavros_msgs::msg::Param::ConstSharedPtr msg); // Updated signature
};
}
#endif  // AVOIDANCE_AVOIDANCE_NODE_H
