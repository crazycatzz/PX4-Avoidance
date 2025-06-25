#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp> // For velocityCallback
#include <mavros_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp> // Was included in ROS1, likely for ground_truth_sub_
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp> // Was included in ROS1
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp> // Was included in ROS1
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/transforms.h> // TF1 specific, remove. Use tf2_eigen or similar for PCL TF.

#include <octomap/OcTree.h>
#include <octomap/octomap.h> // Redundant with OcTree.h? Keep for now.
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h> // For octomap <-> message conversions

// Forward declare AvoidanceNode and WorldVisualizer if their full definition isn't needed here
// This reduces include dependencies. However, if we are creating instances (even shared_ptr),
// we need the full definition. For now, assume full def is needed due to member instances.
#include "avoidance/avoidance_node.h" // Assuming this is now a ROS2 node
#include "global_planner/global_planner.h"

#ifndef DISABLE_SIMULATION
#include <avoidance/rviz_world_loader.h> // Assuming this is now a ROS2 node
#endif

#include <mutex>
#include <string>
#include <vector>
#include <set> // Was included in ROS1
#include <memory> // For std::shared_ptr, std::unique_ptr

namespace global_planner {

struct cameraData {
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_; // Updated type
};

class GlobalPlannerNode : public rclcpp::Node {
 public:
  GlobalPlanner global_planner_; // Algorithm class
  std::vector<GoalCell> waypoints_;

  explicit GlobalPlannerNode(const rclcpp::NodeOptions& options);
  ~GlobalPlannerNode();

 private:
  std::mutex mutex_; // Keep std::mutex

  // Removed ros::NodeHandle members

  // Subscribers
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_full_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_; // Assuming Odometry for ground truth
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr move_base_simple_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sensor_sub_; // Assuming LaserScan
  rclcpp::Subscription<mavros_msgs::msg::Trajectory>::SharedPtr fcu_input_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_temp_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr explored_cells_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_temp_goal_pub_;
  rclcpp::Publisher<mavros_msgs::msg::Trajectory>::SharedPtr mavros_obstacle_free_path_pub_;
  rclcpp::Publisher<mavros_msgs::msg::Trajectory>::SharedPtr mavros_waypoint_publisher_; // Type might need review based on usage
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_waypoint_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_; // For publishing combined/processed clouds

  rclcpp::Time start_time_; // Use rclcpp::Time
  rclcpp::Time last_wp_time_; // Use rclcpp::Time

  rclcpp::TimerBase::SharedPtr cmdloop_timer_;
  rclcpp::TimerBase::SharedPtr plannerloop_timer_;
  // Removed CallbackQueues and AsyncSpinners

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // Removed dynamic_reconfigure::Server

  nav_msgs::msg::Path actual_path_; // Updated type
  geometry_msgs::msg::Point start_pos_; // Updated type
  geometry_msgs::msg::PoseStamped current_goal_; // Updated type
  geometry_msgs::msg::PoseStamped last_goal_;    // Updated type
  geometry_msgs::msg::PoseStamped last_pos_;     // Updated type

  std::vector<geometry_msgs::msg::PoseStamped> last_clicked_points; // Updated type
  std::vector<geometry_msgs::msg::PoseStamped> path_;              // Updated type
  std::vector<cameraData> cameras_; // Contains rclcpp::Subscription now

  int num_octomap_msg_ = 0;
  int num_pos_msg_ = 0;
  double cmdloop_dt_;
  double plannerloop_dt_;
  double mapupdate_dt_; // This was not explicitly initialized or used in provided ROS1 snippet
  double speed_;        // Likely a parameter
  double start_yaw_;
  bool position_received_;
  std::string frame_id_;         // Likely a parameter
  std::string camera_frame_id_;  // Likely a parameter, or part of camera_topics

  // ROS2 Parameters (formerly Dynamic Reconfigure)
  // These will be declared and fetched in constructor, updated by callback
  double clicked_goal_alt_;
  double clicked_goal_radius_;
  bool hover_;
  int simplify_iterations_;
  double simplify_margin_;
  // Add other parameters from GlobalPlannerNodeConfig if they were used by this class

  // For avoidance_node_ and world_visualizer_ consider if they should be components
  // or separate nodes. If separate, interaction is via topics/services.
  // If components, they are managed by a ComponentContainer.
  // For now, using shared_ptr, assuming they are created and managed in .cpp
  std::shared_ptr<avoidance::AvoidanceNode> avoidance_node_ptr_;
#ifndef DISABLE_SIMULATION
  std::shared_ptr<avoidance::WorldVisualizer> world_visualizer_ptr_;
#endif

  void readParams(); // Will change to declare and get ROS2 parameters
  void initializeCameraSubscribers(const std::vector<std::string>& camera_topics); // camera_topics will be a parameter
  void receivePath(const nav_msgs::msg::Path::ConstSharedPtr msg); // Updated signature
  void setNewGoal(const GoalCell& goal);
  void popNextGoal();
  void planPath();
  void setIntermediateGoal();
  bool isCloseToGoal();
  void setCurrentPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses); // Updated type

  // Parameter callback
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  // Topic Callbacks
  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void positionCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void clickedPointCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
  void moveBaseSimpleCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void octomapFullCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr msg);
  void depthCameraCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const std::string& topic_name); // Added topic_name if needed for camera identification
  void fcuInputGoalCallback(const mavros_msgs::msg::Trajectory::ConstSharedPtr msg);

  // Timer Callbacks
  void cmdLoopCallback();
  void plannerLoopCallback();

  void publishGoal(const GoalCell& goal);
  void publishPath();
  void publishSetpoint();
  void printPointInfo(double x, double y, double z);
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
