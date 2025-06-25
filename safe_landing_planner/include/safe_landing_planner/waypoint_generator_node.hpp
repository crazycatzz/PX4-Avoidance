#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp> // For parameter callback

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/trajectory.hpp>
#include <mavros_msgs/msg/position_target.hpp> // For fillUnusedTrajectorySetpoints
#include <std_msgs/msg/float64_multi_array.hpp> // If used by publishers
#include <std_msgs/msg/int64_multi_array.hpp>   // If used by publishers
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// #include <dynamic_reconfigure/server.h> // Removed
#include <safe_landing_planner/msg/slp_grid_msg.hpp> // Updated custom message
// #include <safe_landing_planner/WaypointGeneratorNodeConfig.h> // Removed

#include <safe_landing_planner/waypoint_generator.hpp> // Assumes this is updated or ROS-agnostic enough

#include <Eigen/Dense> // Already in waypoint_generator.hpp but good for clarity
#include <vector>
#include <string>
#include <functional> // For std::bind
#include <memory> // For std::unique_ptr, std::make_shared

namespace avoidance {

class WaypointGeneratorNode final : public rclcpp::Node { // Inherit from rclcpp::Node
 public:
  explicit WaypointGeneratorNode(const rclcpp::NodeOptions& options); // Updated constructor
  ~WaypointGeneratorNode() = default;

  // startNode() logic will be moved into constructor or separate init called by constructor

 protected:
  WaypointGenerator waypointGenerator_; // Algorithm class

  // ros::NodeHandle nh_; // Removed
  // ros::NodeHandle nh_private_; // Removed

  rclcpp::TimerBase::SharedPtr cmdloop_timer_;
  // std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_; // Removed
  // ros::CallbackQueue cmdloop_queue_; // Removed

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<mavros_msgs::msg::Trajectory>::SharedPtr trajectory_sub_; // For goal from FCU mission
  rclcpp::Subscription<safe_landing_planner::msg::SLPGridMsg>::SharedPtr grid_sub_;
  // ros::Subscriber pos_index_sub_; // What type was this? Assuming Int64MultiArray for now if it's index based
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr pos_index_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;

  // Publishers
  rclcpp::Publisher<mavros_msgs::msg::Trajectory>::SharedPtr trajectory_pub_; // For sending waypoints
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr land_hysteresis_pub_; // For visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_goal_pub_; // For visualization

  bool grid_received_ = false;
  double spin_dt_ = 0.1; // Will be a parameter
  Eigen::Vector3f goal_visualization_ = Eigen::Vector3f::Zero();

  // dynamic_reconfigure::Server<safe_landing_planner::WaypointGeneratorNodeConfig> server_; // Removed

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
  * @brief     callaback for setting the goal from the FCU Mission Waypoints
  * @param[in] msg, current and next position goals
  **/
  void trajectoryCallback(const mavros_msgs::msg::Trajectory::ConstSharedPtr msg); // Updated signature

  /**
  * @brief     callaback with the grid calculated by the safe_landing_planner
  * @param[in] msg, grid
  **/
  void gridCallback(const safe_landing_planner::msg::SLPGridMsg::ConstSharedPtr msg); // Updated signature

  /**
  * @brief     callaback with the vehicle state
  * @param[in] msg, FCU vehicle state
  **/
  void stateCallback(const mavros_msgs::msg::State::ConstSharedPtr msg); // Updated signature

  /**
  * @brief     publishes the computed waypoints to the FCU
  * @param[in] pos_sp, position setpoint
  * @param[in] vel_sp, velocity setpoint
  * @param[in] yaw_sp, yaw setpoint
  * @param[in] yaw_speed_sp, yaw speed setpoint
  **/
  void publishTrajectorySetpoints(const Eigen::Vector3f& pos_sp, const Eigen::Vector3f& vel_sp, float yaw_sp,
                                  float yaw_speed_sp);

  /**
  * @brief     fills unused waypoints with NANs
  * @param[in] point, unused waypoints
  **/
  void fillUnusedTrajectorySetpoints(mavros_msgs::PositionTarget& point);

  /**
  * @brief     visualize landing area decision grid in Rviz
  **/
  void landingAreaVisualization();

  /**
  * @brief     visualize goal in Rviz
  **/
  void goalVisualization();
};
}
