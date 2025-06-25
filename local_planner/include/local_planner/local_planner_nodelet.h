#ifndef LOCAL_PLANNER_LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_LOCAL_PLANNER_NODE_H

#include "avoidance/transform_buffer.h"
#include "local_planner/avoidance_output.h"         // Uses rclcpp::Time
#include "local_planner/local_planner_visualization.h" // Uses rclcpp::Node::SharedPtr, msgs

#ifndef DISABLE_SIMULATION
#include "avoidance/rviz_world_loader.h" // Will be rclcpp::Node
#endif

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_adapter.hpp> // For adapting nodelet-like structure if needed
#include <rclcpp_components/register_node_macro.hpp> // For RCLCPP_COMPONENTS_REGISTER_NODE

// ROS2 Message Includes
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp> // For velocity callback
#include <geometry_msgs/msg/pose_stamped.hpp> // For goal callback
#include <mavros_msgs/msg/altitude.hpp>
#include <mavros_msgs/msg/companion_process_status.hpp>
#include <mavros_msgs/srv/set_mode.hpp> // Assuming SetMode is a service
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/trajectory.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp> // For position callback if it uses Odometry

// TF2 Includes
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp> // For Eigen conversions with TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For TF2 and geometry_msgs conversions

// PCL Includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg, toROSMsg
// #include <pcl_ros/transforms.h> // This is TF1 specific, remove. Use tf2_eigen and pcl::transformPointCloud

#include <Eigen/Core>
// #include <boost/bind.hpp> // Use std::bind or lambdas

#include <avoidance/common.h> // Already updated
// #include <dynamic_reconfigure/server.h> // Removed
// #include <local_planner/LocalPlannerNodeConfig.h> // Removed
#include "avoidance/avoidance_node.h" // Assumed to be a ROS2 node class now

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace avoidance {

class LocalPlanner;
class WaypointGenerator;

struct cameraData {
  std::string topic_;
  ros::Subscriber pointcloud_sub_;

  pcl::PointCloud<pcl::PointXYZ> untransformed_cloud_;
  bool received_;

  pcl::PointCloud<pcl::PointXYZ> transformed_cloud_;
  bool transformed_;

  std::unique_ptr<std::mutex> camera_mutex_;
  std::unique_ptr<std::condition_variable> camera_cv_;

  bool transform_registered_ = false;
  std::thread transform_thread_;

  FOV fov_fcu_frame_;
};

class LocalPlannerNodelet : public rclcpp_components::Node { // Inherit from rclcpp_components::Node
 public:
  explicit LocalPlannerNodelet(const rclcpp::NodeOptions& options); // Updated constructor
  virtual ~LocalPlannerNodelet();
  // onInit() is replaced by constructor logic in ROS2 components
  // InitializeNodelet() content will be moved to constructor or other init methods

  std::atomic<bool> should_exit_{false};

  std::unique_ptr<LocalPlanner> local_planner_;
  std::unique_ptr<WaypointGenerator> wp_generator_;
  // std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_; // Removed, use ROS2 executors

  std::thread worker; // Keep for now, review its purpose
  std::thread worker_tf_listener; // Keep for now, review its purpose

  LocalPlannerVisualization visualizer_; // Needs to be initialized with a Node::SharedPtr

  // Changed to shared_ptr, instantiation needs review (e.g. in constructor, pass options)
  std::shared_ptr<avoidance::AvoidanceNode> avoidance_node_ptr_;

#ifndef DISABLE_SIMULATION
  std::shared_ptr<WorldVisualizer> world_visualizer_ptr_;
#endif

  std::mutex running_mutex_;  ///< guard against concurrent access to input &
                              /// output data (point_cloud, position, ...)

  bool position_received_ = false;

  /**
  * @brief     handles threads for data publication and subscription
  **/
  void threadFunction(); // Review if still needed or if ROS2 executor handles this

  /**
  * @brief     start spinners
  **/
  // void startNode(); // Spinners removed

  /**
  * @brief     updates the local planner agorithm with the latest pointcloud,
  *            vehicle position, velocity, state, and distance to ground, goal,
  *            setpoint sent to the FCU
  **/
  void updatePlannerInfo();

  /**
  * @brief     computes the number of transformed pointclouds
  * @ returns  number of transformed pointclouds
  **/
  size_t numTransformedClouds();

  /**
  * @brief     threads for transforming pointclouds
  **/
  void pointCloudTransformThread(int index);

  /**
  * @brief      calculates position and velocity setpoints and sends to the FCU
  * @param[in]  hover, true if the vehicle is loitering
  **/
  void calculateWaypoints(bool hover);

  /**
  * @brief      sends out a status to the FCU which will be received as a
  *heartbeat
  **/
  void publishSystemStatus();

  /**
  * @brief      set avoidance system status
  **/
  void setSystemStatus(MAV_STATE state);

  /**
  * @brief      get avoidance system status
  **/
  MAV_STATE getSystemStatus();

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
  void checkFailsafe(rclcpp::Duration since_last_cloud, rclcpp::Duration since_start, bool& hover); // Updated types

  /**
  * @brief     safes received transforms to buffer;
  **/
  void transformBufferThread(); // Review if still needed with tf2_ros::Buffer

 private:
  // avoidance::LocalPlannerNodeConfig rqt_param_config_; // Removed, parameters handled by ROS2 param system

  // ros::NodeHandle nh_; // Removed
  // ros::NodeHandle nh_private_; // Removed

  rclcpp::TimerBase::SharedPtr cmdloop_timer_;
  // ros::CallbackQueue cmdloop_queue_; // Removed

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pos_setpoint_pub_; // Assuming PoseStamped
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr mavros_vel_setpoint_pub_; // Assuming TwistStamped
  rclcpp::Publisher<mavros_msgs::msg::Trajectory>::SharedPtr mavros_obstacle_free_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr mavros_obstacle_distance_pub_; // Assuming LaserScan
  rclcpp::Publisher<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr mavros_system_status_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clicked_goal_sub_; // For /move_base_simple/goal
  rclcpp::Subscription<mavros_msgs::msg::Trajectory>::SharedPtr fcu_input_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_topic_sub_; // For external goal topic
  rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr distance_sensor_sub_; // Assuming Altitude msg for range

  // ros::CallbackQueue pointcloud_queue_; // Removed, use callback groups if needed
  // ros::CallbackQueue main_queue_; // Removed

  rclcpp::Time start_time_;     // Updated type
  rclcpp::Time last_wp_time_;   // Updated type
  rclcpp::Time t_status_sent_;  // Updated type

  geometry_msgs::msg::PoseStamped goal_mission_item_msg_; // Updated type
  mavros_msgs::msg::Altitude ground_distance_msg_;      // Updated type

  bool new_goal_ = false;

  std::mutex waypoints_mutex_; // Keep std::mutex
  Eigen::Vector3f newest_waypoint_position_;
  Eigen::Vector3f last_waypoint_position_;
  Eigen::Vector3f newest_adapted_waypoint_position_;
  Eigen::Vector3f last_adapted_waypoint_position_;
  Eigen::Vector3f newest_position_;
  Eigen::Quaternionf newest_orientation_;
  Eigen::Vector3f last_position_;
  Eigen::Quaternionf last_orientation_;
  Eigen::Vector3f velocity_;
  Eigen::Vector3f desired_velocity_;
  Eigen::Vector3f goal_position_;
  Eigen::Vector3f prev_goal_position_;

  NavigationState nav_state_ = NavigationState::none; // From avoidance::common.h

  // dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig>* server_ = nullptr; // Removed
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_; // New TF2 buffer
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // New TF2 listener
  avoidance::tf_buffer::TransformBuffer custom_tf_buffer_; // The custom buffer (already updated)
  std::condition_variable tf_buffer_cv_; // Keep std::condition_variable

  std::mutex buffered_transforms_mutex_; // Keep std::mutex
  std::vector<std::pair<std::string, std::string>> buffered_transforms_;

  std::mutex transformed_cloud_mutex_; // Keep std::mutex
  std::condition_variable transformed_cloud_cv_; // Keep std::condition_variable

  std::vector<cameraData> cameras_; // cameraData struct now holds rclcpp::Subscription

  bool armed_ = false;
  bool data_ready_ = false;
  bool hover_; // This will be a ROS2 parameter
  bool planner_is_healthy_;
  bool position_not_received_error_sent_ = false;
  bool callPx4Params_; // This logic might need review with ROS2 params
  bool accept_goal_input_topic_; // This will be a ROS2 parameter
  bool is_land_waypoint_{false};
  bool is_takeoff_waypoint_{false};
  double spin_dt_; // This will be a ROS2 parameter (if used for a timer)
  int path_length_ = 0;
  std::vector<float> algo_time;

  float desired_yaw_setpoint_{NAN};
  float desired_yaw_speed_setpoint_{NAN};

  // boost::recursive_mutex config_mutex_; // std::recursive_mutex can be used if needed, or simplify locking

  // Parameter callback
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  /**
  * @brief     subscribes to all the camera topics and camera info
  * @param     camera_topics, array with the pointcloud topics strings
  **/
  void initializeCameraSubscribers(const std::vector<std::string>& camera_topics); // Parameter will be std::vector<std::string>

  /**
  * @brief     callaback for vehicle position and orientation
  * @param[in] msg, vehicle position and orientation in ENU frame
  **/
  void positionCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg); // Updated signature

  /**
  * @brief     callaback for pointcloud
  * @param[in] msg, pointcloud message
  * @param[in] index, pointcloud instance number
  **/
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, int index); // Updated signature
  /**
  * @brief     callaback for camera information
  * @param[in] msg, camera information message
  * @param[in] index, camera info instace number
  **/
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, int index); // Updated signature

  /**
  * @brief     callaback for vehicle velocity
  * @param[in] msg, vehicle velocity message
  **/
  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg); // Updated signature

  /**
  * @brief     callaback for vehicle state
  * @param[in] msg, vehicle position and orientation in ENU frame
  **/
  void stateCallback(const mavros_msgs::msg::State::ConstSharedPtr msg); // Updated signature
  void cmdLoopCallback(); // Updated signature (no TimerEvent)

  /**
  * @brief     reads parameters from launch file and yaml file
  **/
  void readParams(); // Will change to declare and get ROS2 parameters

  /**
  * @brief     callaback for clicking cells in the polar histogram
  * @param[in] msg, vehicle position and orientation in ENU frame
  **/
  void clickedPointCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg); // Updated signature
  /**
  * @brief     callaback for selecting the goal by cliking on the position in
  *the Rviz visualization of the world
  * @param[in] msg, goal position
  **/
  void clickedGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg); // Updated signature
  /**
  * @brief     callaback
  * @param[in] msg,
  **/
  void updateGoalCallback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg); // Updated signature
  /**
  * @brief     callaback for setting the goal from the FCU Mission Waypoints
  * @param[in] msg, current and next position goals
  **/
  void fcuInputGoalCallback(const mavros_msgs::msg::Trajectory::ConstSharedPtr msg); // Updated signature
  /**
  * @brief     callaback for distance to the ground
  * @param[in] msg, altitude message
  **/
  void distanceSensorCallback(const mavros_msgs::msg::Altitude::ConstSharedPtr msg); // Updated signature

  /**
  * @brief     callaback for vehicle state - DUPLICATE? Seems same as stateCallback above.
  * @param[in] msg, vehicle position and orientation in ENU frame
  **/
  void printPointInfo(double x, double y, double z); // No ROS types

  /**
  * @brief     sends out emulated LaserScan data to the flight controller
  **/
  void publishLaserScan() const;
};
}
#endif  // LOCAL_PLANNER_LOCAL_PLANNER_NODE_H
