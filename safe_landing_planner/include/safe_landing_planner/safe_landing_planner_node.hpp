#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#ifndef DISABLE_SIMULATION
#include "avoidance/rviz_world_loader.h" // Assumes this is updated to be a ROS2 node/component
#endif

#include <avoidance/common.h> // Assumes this is updated (uses rclcpp::Time etc)
// #include <dynamic_reconfigure/server.h> // Removed
#include <geometry_msgs/msg/pose_stamped.hpp> // Updated
#include <mavros_msgs/msg/companion_process_status.hpp> // Updated
#include <sensor_msgs/msg/point_cloud2.hpp> // Updated
#include <safe_landing_planner/msg/slp_grid_msg.hpp> // Updated custom message

#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h> // For PCL <-> ROS2 message conversions
// #include <pcl_ros/point_cloud.h> // Not always needed if using sensor_msgs::msg::PointCloud2 directly with pcl_conversions
// #include <pcl_ros/transforms.h> // TF1 specific, use tf2_eigen and pcl::transformPointCloud

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>


#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <memory> // For std::unique_ptr, std::shared_ptr

#include "safe_landing_planner.hpp" // Assumes this is updated
#include "safe_landing_planner_visualization.hpp" // Assumes this is updated

namespace avoidance {

class SafeLandingPlannerNode : public rclcpp::Node { // Inherit from rclcpp::Node
 public:
  explicit SafeLandingPlannerNode(const rclcpp::NodeOptions& options); // Updated constructor
  ~SafeLandingPlannerNode(); // Default is fine if threads are joined properly

  std::unique_ptr<SafeLandingPlanner> safe_landing_planner_;

#ifndef DISABLE_SIMULATION
  std::shared_ptr<avoidance::WorldVisualizer> world_visualizer_ptr_; // Changed to shared_ptr
#endif

  std::atomic<bool> should_exit_{false};

  std::thread worker_; // Keep worker thread

  // startNode() logic will be moved into constructor or a separate init method

  /**
  * @brief     threads for transforming pointclouds
  **/
  void pointCloudTransformThread(); // Keep this thread function

 private:
  rclcpp::TimerBase::SharedPtr cmdloop_timer_; // Updated type

  std::unique_ptr<std::mutex> cloud_msg_mutex_; // Keep std::mutex
  std::unique_ptr<std::mutex> transformed_cloud_mutex_; // Keep std::mutex
  std::unique_ptr<std::condition_variable> cloud_ready_cv_; // Keep std::condition_variable
  // std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_; // Removed
  // ros::CallbackQueue cmdloop_queue_; // Removed

  sensor_msgs::msg::PointCloud2 newest_cloud_msg_; // Updated type

  // ros::NodeHandle nh_; // Removed

  // Publishers
  rclcpp::Publisher<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr mavros_system_status_pub_;
  rclcpp::Publisher<safe_landing_planner::msg::SLPGridMsg>::SharedPtr grid_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<safe_landing_planner::msg::SLPGridMsg>::SharedPtr raw_grid_sub_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseStamped current_pose_;  // Updated type
  geometry_msgs::msg::PoseStamped previous_pose_; // Updated type
  mavros_msgs::msg::CompanionProcessStatus status_msg_; // Updated type

  rclcpp::Time start_time_;            // Updated type
  rclcpp::Time last_algo_time_;        // Updated type
  rclcpp::Time t_status_sent_;         // Updated type

  SafeLandingPlannerVisualization visualizer_; // Needs to be initialized with Node::SharedPtr

  bool position_received_ = false;
  bool cloud_transformed_ = false;
  double spin_dt_ = 0.1; // Will be a parameter

  // dynamic_reconfigure::Server<safe_landing_planner::SafeLandingPlannerNodeConfig> server_; // Removed
  // safe_landing_planner::SafeLandingPlannerNodeConfig rqt_param_config_; // Removed
  // Parameters will be direct members if needed, or accessed via get_parameter()

  // Parameter callback
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  /**
  * @brief main loop callback
  **/
  void cmdLoopCallback(); // No TimerEvent

  /**
  * @brif callback for vehicle position and orientation
  * @param[in] msg, pose message coming fro the FCU
  **/
  void positionCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg); // Updated signature
  /**
  * @brif callback for pointcloud
  * @param[in] msg, current frame pointcloud
  **/
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg); // Updated signature

  /**
  * @brief     callaback for grid coming from rosbag
  * @param     msg, SLPGridMsg message
  **/
  void rawGridCallback(const safe_landing_planner::msg::SLPGridMsg::ConstSharedPtr msg); // Updated signature
  /**
  * @brif     sends out a status to the FCU which will be received as a
  *heartbeat
  **/
  void publishSystemStatus();

  /**
  * @brief      check healthiness of the avoidance system to trigger failsafe in
  *the FCU
  * @param[in]  since_last_algo, time elapsed since the last iteration of the
  *landing site detection algorithm
  * @param[in]  since_start, time elapsed since staring the node
  **/
  void checkFailsafe(ros::Duration since_last_algo, ros::Duration since_start);

  /**
  * @brief      publishes the computed grid fot the waypoint_generator_node to
  *use
  **/
  void publishSerialGrid();
};
}
