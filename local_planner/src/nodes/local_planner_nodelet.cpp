#include "local_planner/local_planner_nodelet.h"

#include "local_planner/local_planner.h"
#include "local_planner/planner_functions.h"
#include "local_planner/tree_node.h"
#include "local_planner/waypoint_generator.h"

#include <boost/algorithm/string.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>
#include <functional>

#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>
#include <sensor_msgs/image_encodings.hpp>


namespace avoidance {

LocalPlannerNodelet::LocalPlannerNodelet(const rclcpp::NodeOptions& options)
    : rclcpp_components::Node("local_planner_nodelet", options),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      custom_tf_buffer_(5.0f),
      spin_dt_(0.1)
{
  RCLCPP_INFO(this->get_logger(), "Initializing LocalPlannerNodelet (ROS2)...");

  local_planner_ = std::make_unique<LocalPlanner>();
  wp_generator_ = std::make_unique<WaypointGenerator>();

  // avoidance_node_ptr_ and world_visualizer_ptr_ are initialized to nullptr by default in header.
  // Instantiation of other nodes is complex and typically handled by launch files/composition in ROS2.
  // For this port, we are focusing on this node's functionality.
  // If these were critical for LocalPlannerNodelet to function directly, they would need separate lifecycle management.
  /*
  rclcpp::NodeOptions sub_node_options;
  avoidance_node_ptr_ = std::make_shared<avoidance::AvoidanceNode>(sub_node_options);
  #ifndef DISABLE_SIMULATION
  world_visualizer_ptr_ = std::make_shared<WorldVisualizer>(sub_node_options, this->get_name());
  #endif
  */

  this->readParamsAndInitInterfaces();

  last_wp_time_ = this->now();
  start_time_ = this->now();
  position_received_ = false;
  position_not_received_error_sent_ = false;
  new_goal_ = true;
  hover_ = this->get_parameter("hover").as_bool();
  planner_is_healthy_ = true;
  armed_ = false;


  worker = std::thread(&LocalPlannerNodelet::threadFunction, this);
  worker_tf_listener = std::thread(&LocalPlannerNodelet::transformBufferThread, this);

  RCLCPP_INFO(this->get_logger(), "LocalPlannerNodelet initialized.");
}

LocalPlannerNodelet::~LocalPlannerNodelet() {
  RCLCPP_INFO(this->get_logger(), "Destroying LocalPlannerNodelet...");
  should_exit_ = true;
  {
    std::lock_guard<std::mutex> guard(buffered_transforms_mutex_);
    tf_buffer_cv_.notify_all();
  }
  {
    std::lock_guard<std::mutex> guard(transformed_cloud_mutex_);
    transformed_cloud_cv_.notify_all();
  }

  for (size_t i = 0; i < cameras_.size(); ++i) {
    if (cameras_[i].camera_cv_) {
        std::lock_guard<std::mutex> guard(*cameras_[i].camera_mutex_);
        cameras_[i].camera_cv_->notify_all();
    }
    if (cameras_[i].transform_thread_.joinable()) cameras_[i].transform_thread_.join();
  }

  if (worker.joinable()) worker.join();
  if (worker_tf_listener.joinable()) worker_tf_listener.join();
  RCLCPP_INFO(this->get_logger(), "LocalPlannerNodelet threads joined.");
}


void LocalPlannerNodelet::readParamsAndInitInterfaces() {
    this->declare_parameter<double>("goal_x_param", 0.0);
    this->declare_parameter<double>("goal_y_param", 0.0);
    this->declare_parameter<double>("goal_z_param", 3.0);
    this->declare_parameter<bool>("accept_goal_input_topic", false);
    this->declare_parameter<std::vector<std::string>>("pointcloud_topics", std::vector<std::string>());
    this->declare_parameter<double>("spin_dt", 0.02);

    this->declare_parameter<int>("children_per_node", 1);
    this->declare_parameter<int>("n_expanded_nodes", 5);
    this->declare_parameter<double>("tree_node_distance", 1.0);
    this->declare_parameter<double>("max_path_length", 4.0);
    this->declare_parameter<double>("smoothing_margin_degrees", 30.0);
    this->declare_parameter<double>("tree_heuristic_weight", 10.0);
    this->declare_parameter<double>("max_sensor_range", 12.0);
    this->declare_parameter<double>("min_sensor_range", 0.2);
    this->declare_parameter<double>("max_point_age_s", 10.0);
    this->declare_parameter<int>("min_num_points_per_cell", 3);
    this->declare_parameter<double>("timeout_startup", 20.0);
    this->declare_parameter<double>("timeout_critical", 0.5);
    this->declare_parameter<double>("timeout_termination", 20.0);
    this->declare_parameter<double>("pitch_cost_param", 3.0);
    this->declare_parameter<double>("yaw_cost_param", 0.5);
    this->declare_parameter<double>("velocity_cost_param", 1.5);
    this->declare_parameter<double>("obstacle_cost_param", 5.0);
    this->declare_parameter<double>("smoothing_speed_xy", 10.0);
    this->declare_parameter<double>("smoothing_speed_z", 3.0);
    this->declare_parameter<bool>("hover", false);

    // Apply all parameters by calling the callback once
    this->parametersCallback(this->get_parameters(this->list_parameters({}, 0).names));
    auto param_cb = std::bind(&LocalPlannerNodelet::parametersCallback, this, std::placeholders::_1);
    this->add_on_set_parameters_callback(param_cb);

    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    rclcpp::QoS latching_qos(rclcpp::KeepLast(1));
    latching_qos.transient_local();

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", qos_profile, std::bind(&LocalPlannerNodelet::positionCallback, this, std::placeholders::_1));
    velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "mavros/local_position/velocity_local", qos_profile, std::bind(&LocalPlannerNodelet::velocityCallback, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", qos_profile, std::bind(&LocalPlannerNodelet::stateCallback, this, std::placeholders::_1));
    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "clicked_point", qos_profile, std::bind(&LocalPlannerNodelet::clickedPointCallback, this, std::placeholders::_1));
    clicked_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "move_base_simple/goal", qos_profile, std::bind(&LocalPlannerNodelet::clickedGoalCallback, this, std::placeholders::_1));
    fcu_input_sub_ = this->create_subscription<mavros_msgs::msg::Trajectory>(
        "mavros/trajectory/desired", qos_profile, std::bind(&LocalPlannerNodelet::fcuInputGoalCallback, this, std::placeholders::_1));
    goal_topic_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "input/goal_position", qos_profile, std::bind(&LocalPlannerNodelet::updateGoalCallback, this, std::placeholders::_1));
    distance_sensor_sub_ = this->create_subscription<mavros_msgs::msg::Altitude>(
        "mavros/altitude", qos_profile, std::bind(&LocalPlannerNodelet::distanceSensorCallback, this, std::placeholders::_1));

    mavros_pos_setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", qos_profile);
    mavros_vel_setpoint_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", qos_profile);
    mavros_obstacle_free_path_pub_ = this->create_publisher<mavros_msgs::msg::Trajectory>("mavros/trajectory/generated", qos_profile);
    mavros_obstacle_distance_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("mavros/obstacle/send", qos_profile);
    mavros_system_status_pub_ = this->create_publisher<mavros_msgs::msg::CompanionProcessStatus>("mavros/companion_process/status", latching_qos);

    visualizer_.initializePublishers(std::dynamic_pointer_cast<rclcpp::Node>(this->get_node_base_interface()));

    std::vector<std::string> camera_topics_vec;
    this->get_parameter("pointcloud_topics", camera_topics_vec);
    initializeCameraSubscribers(camera_topics_vec);

    if (local_planner_) local_planner_->applyGoal();
    setSystemStatus(MAV_STATE::MAV_STATE_BOOT);

    cmdloop_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(spin_dt_), std::bind(&LocalPlannerNodelet::cmdLoopCallback, this));
}

rcl_interfaces::msg::SetParametersResult LocalPlannerNodelet::parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    std::lock_guard<std::mutex> guard(running_mutex_);

    costParameters cost_params_update = local_planner_->cost_params_;
    int new_children_per_node = local_planner_->children_per_node_;
    int new_n_expanded_nodes = local_planner_->n_expanded_nodes_;
    float new_min_sensor_range = local_planner_->min_sensor_range_;
    float new_max_sensor_range = local_planner_->max_sensor_range_;
    float new_smoothing_margin_degrees = local_planner_->smoothing_margin_degrees_;
    float new_max_point_age_s = local_planner_->max_point_age_s_;
    float current_speed = local_planner_->speed_;

    float sp_tree_node_distance = NAN, sp_max_path_length = NAN, sp_heuristic_weight = NAN;
    if (local_planner_ && local_planner_->star_planner_){
        sp_tree_node_distance = local_planner_->star_planner_->tree_node_distance_;
        sp_max_path_length = local_planner_->star_planner_->max_path_length_;
        sp_heuristic_weight = local_planner_->star_planner_->tree_heuristic_weight_;
    }


    for (const auto &param : parameters) {
        const std::string &name = param.get_name();
        if (name == "goal_x_param") goal_position_.x() = static_cast<float>(param.as_double());
        else if (name == "goal_y_param") goal_position_.y() = static_cast<float>(param.as_double());
        else if (name == "goal_z_param") {
            float old_z = goal_position_.z();
            goal_position_.z() = static_cast<float>(param.as_double());
            if (local_planner_ && std::abs(old_z - goal_position_.z()) > 1e-3) {
                 local_planner_->setGoal(goal_position_); new_goal_ = true;
            }
        }
        else if (name == "accept_goal_input_topic") accept_goal_input_topic_ = param.as_bool();
        else if (name == "spin_dt") { // TODO: Recreate cmdloop_timer if this changes. Not trivial.
            spin_dt_ = param.as_double();
            RCLCPP_WARN(this->get_logger(), "spin_dt changed, timer period update isn't implemented yet.");
        }
        else if (name == "children_per_node") new_children_per_node = param.as_int();
        else if (name == "n_expanded_nodes") new_n_expanded_nodes = param.as_int();
        else if (name == "tree_node_distance") sp_tree_node_distance = static_cast<float>(param.as_double());
        else if (name == "max_path_length") sp_max_path_length = static_cast<float>(param.as_double());
        else if (name == "smoothing_margin_degrees") new_smoothing_margin_degrees = static_cast<float>(param.as_double());
        else if (name == "tree_heuristic_weight") sp_heuristic_weight = static_cast<float>(param.as_double());
        else if (name == "max_sensor_range") new_max_sensor_range = static_cast<float>(param.as_double());
        else if (name == "min_sensor_range") new_min_sensor_range = static_cast<float>(param.as_double());
        else if (name == "max_point_age_s") new_max_point_age_s = static_cast<float>(param.as_double());
        else if (name == "min_num_points_per_cell" && local_planner_) local_planner_->min_num_points_per_cell_ = param.as_int();
        else if (name == "timeout_startup" && local_planner_) local_planner_->timeout_startup_ = param.as_double();
        else if (name == "timeout_critical" && local_planner_) local_planner_->timeout_critical_ = param.as_double();
        else if (name == "timeout_termination" && local_planner_) local_planner_->timeout_termination_ = param.as_double();
        else if (name == "pitch_cost_param") cost_params_update.pitch_cost_param = static_cast<float>(param.as_double());
        else if (name == "yaw_cost_param") cost_params_update.yaw_cost_param = static_cast<float>(param.as_double());
        else if (name == "velocity_cost_param") cost_params_update.velocity_cost_param = static_cast<float>(param.as_double());
        else if (name == "obstacle_cost_param") cost_params_update.obstacle_cost_param = static_cast<float>(param.as_double());
        else if (name == "smoothing_speed_xy" && wp_generator_) wp_generator_->setSmoothingSpeed(static_cast<float>(param.as_double()), wp_generator_->smoothing_speed_z_);
        else if (name == "smoothing_speed_z" && wp_generator_) wp_generator_->setSmoothingSpeed(wp_generator_->smoothing_speed_xy_, static_cast<float>(param.as_double()));
        else if (name == "hover") hover_ = param.as_bool();
        else if (name == "pointcloud_topics") {
             RCLCPP_WARN(this->get_logger(), "'pointcloud_topics' changed dynamically. Re-initializing camera subscribers.");
             initializeCameraSubscribers(param.as_string_array());
        }
        else { RCLCPP_WARN(this->get_logger(), "Unknown parameter in callback: %s", name.c_str()); }
    }

    if (local_planner_) {
        local_planner_->updateAlgorithmParams( new_children_per_node, new_n_expanded_nodes,
            new_min_sensor_range, new_max_sensor_range, new_smoothing_margin_degrees, new_max_point_age_s,
            current_speed, cost_params_update);
        if (local_planner_->star_planner_) {
             local_planner_->star_planner_->updateStarPlannerParams( new_children_per_node, new_n_expanded_nodes,
                sp_tree_node_distance, sp_max_path_length, new_smoothing_margin_degrees, sp_heuristic_weight,
                new_max_sensor_range, new_min_sensor_range);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Parameters reconfigured.");
    return result;
}

void LocalPlannerNodelet::initializeCameraSubscribers(const std::vector<std::string>& camera_topics) {
  cameras_.clear();
  cameras_.resize(camera_topics.size());
  for (size_t i = 0; i < camera_topics.size(); i++) {
    cameras_[i].camera_mutex_ = std::make_unique<std::mutex>();
    cameras_[i].camera_cv_ = std::make_unique<std::condition_variable>();
    cameras_[i].received_ = false;
    cameras_[i].transformed_ = false;

    auto callback = [this, index=i](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        this->pointCloudCallback(msg, index);
    };
    cameras_[i].pointcloud_sub_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(camera_topics[i], rclcpp::SensorDataQoS(), callback);
    cameras_[i].topic_ = camera_topics[i];
    cameras_[i].transform_thread_ = std::thread(&LocalPlannerNodelet::pointCloudTransformThread, this, i);
    RCLCPP_INFO(this->get_logger(), "Subscribed to pointcloud topic: %s", camera_topics[i].c_str());
  }
}

size_t LocalPlannerNodelet::numTransformedClouds() {
  size_t num_transformed_clouds = 0;
  for (size_t i = 0; i < cameras_.size(); i++) {
    std::lock_guard<std::mutex> transformed_cloud_guard(*(cameras_[i].camera_mutex_));
    if (cameras_[i].transformed_) num_transformed_clouds++;
  }
  return num_transformed_clouds;
}

void LocalPlannerNodelet::updatePlannerInfo() {
  if (!local_planner_ || !wp_generator_) return;

  local_planner_->original_cloud_vector_.assign(cameras_.size(), pcl::PointCloud<pcl::PointXYZ>());
  for (size_t i = 0; i < cameras_.size(); ++i) {
    std::lock_guard<std::mutex> transformed_cloud_guard(*(cameras_[i].camera_mutex_));
    if (cameras_[i].transformed_) {
        local_planner_->original_cloud_vector_[i] = cameras_[i].transformed_cloud_;
        cameras_[i].transformed_cloud_.clear(); // Clear after copying
        cameras_[i].transformed_ = false;
    }
    local_planner_->setFOV(i, cameras_[i].fov_fcu_frame_);
    wp_generator_->setFOV(i, cameras_[i].fov_fcu_frame_);
  }

  local_planner_->setState(newest_position_, velocity_, newest_orientation_);
  local_planner_->currently_armed_ = armed_;
  if (new_goal_) {
    local_planner_->setGoal(goal_position_);
    local_planner_->setPreviousGoal(prev_goal_position_);
    new_goal_ = false;
  }
  local_planner_->last_sent_waypoint_ = newest_waypoint_position_;

  // local_planner_->px4_ parameters and mission_item_speed_ are set by the parameter callback now.
  // If avoidance_node_ptr_ was used, this would be:
  // if(avoidance_node_ptr_) {
  //    local_planner_->px4_ = avoidance_node_ptr_->getPX4Parameters();
  //    local_planner_->mission_item_speed_ = avoidance_node_ptr_->getMissionItemSpeed();
  // }
}

void LocalPlannerNodelet::positionCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> guard(running_mutex_);
  last_position_ = newest_position_;
  newest_position_ = toEigen(msg->pose.position);
  newest_orientation_ = toEigen(msg->pose.orientation);
  position_received_ = true;
}

void LocalPlannerNodelet::velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> guard(running_mutex_);
  velocity_ = toEigen(msg->twist.linear);
}

void LocalPlannerNodelet::stateCallback(const mavros_msgs::msg::State::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> guard(running_mutex_);
  armed_ = msg->armed;
  std::string mode = msg->mode;
  if (mode == "AUTO.MISSION") nav_state_ = NavigationState::mission;
  else if (mode == "AUTO.TAKEOFF") nav_state_ = NavigationState::auto_takeoff;
  else if (mode == "AUTO.LAND") nav_state_ = NavigationState::auto_land;
  else if (mode == "AUTO.RTL") nav_state_ = NavigationState::auto_rtl;
  else if (mode == "AUTO.RTGS") nav_state_ = NavigationState::auto_rtgs;
  else if (mode == "AUTO.LOITER") nav_state_ = NavigationState::auto_loiter;
  else if (mode == "OFFBOARD") nav_state_ = NavigationState::offboard;
  else nav_state_ = NavigationState::none;
}

void LocalPlannerNodelet::cmdLoopCallback() {
  std::unique_lock<std::mutex> waypoints_lock(waypoints_mutex_); // Use unique_lock for condition variable if needed later
  bool current_hover_state = hover_; // Read hover state under lock if it can be changed by param callback
  waypoints_lock.unlock(); // Unlock before potentially long operations or logging

  if (!position_received_ && rclcpp::ok()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "\033[1;33m Planner abort: missing required data from FCU \n \033[0m"
                         "Local planner has not received a position from FCU, check connections and FCU parameters.");
    setSystemStatus(MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
    position_not_received_error_sent_ = true;
    return;
  }

  rclcpp::Time now = this->now();
  rclcpp::Duration since_last_cloud = now - last_wp_time_;
  rclcpp::Duration since_start = now - start_time_;

  checkFailsafe(since_last_cloud, since_start, current_hover_state); // Pass potentially updated hover state

  MAV_STATE current_system_status = getSystemStatus();
  if (current_system_status == MAV_STATE::MAV_STATE_ACTIVE || current_system_status == MAV_STATE::MAV_STATE_CRITICAL) {
    calculateWaypoints(current_hover_state);
  }
  // position_received_ = false; // Resetting this here seems problematic for continuous operation
}

void LocalPlannerNodelet::setSystemStatus(MAV_STATE state) {
  // If avoidance_node_ptr_ is used:
  // if(avoidance_node_ptr_) avoidance_node_ptr_->setSystemStatus(state);
  // Else, if this node manages status directly (which seems to be the case from header):
  companion_state_ = state; // Assuming companion_state_ is a member of this class
}

MAV_STATE LocalPlannerNodelet::getSystemStatus() {
  // If avoidance_node_ptr_ is used:
  // if(avoidance_node_ptr_) return avoidance_node_ptr_->getSystemStatus();
  // return MAV_STATE::MAV_STATE_UNINIT; // Default if no avoidance_node
  return companion_state_; // Assuming companion_state_ is a member
}

void LocalPlannerNodelet::calculateWaypoints(bool hover_flag) {
  std::lock_guard<std::mutex> guard(running_mutex_);
  bool is_airborne = armed_ && (nav_state_ != NavigationState::none);

  if (!wp_generator_ || !local_planner_) return;

  wp_generator_->updateState(newest_position_, newest_orientation_, goal_position_, prev_goal_position_, velocity_,
                             hover_flag, is_airborne, nav_state_, is_land_waypoint_, is_takeoff_waypoint_,
                             desired_velocity_);
  waypointResult result = wp_generator_->getWaypoints();

  Eigen::Vector3f closest_pt, deg60_pt;
  wp_generator_->getOfftrackPointsForVisualization(closest_pt, deg60_pt);

  last_waypoint_position_ = newest_waypoint_position_;
  newest_waypoint_position_ = result.smoothed_goto_position;
  last_adapted_waypoint_position_ = newest_adapted_waypoint_position_;
  newest_adapted_waypoint_position_ = result.adapted_goto_position;

  visualizer_.visualizeWaypoints(result.goto_position, result.adapted_goto_position, result.smoothed_goto_position);
  visualizer_.publishPaths(last_position_, newest_position_, last_waypoint_position_, newest_waypoint_position_,
                           last_adapted_waypoint_position_, newest_adapted_waypoint_position_);
  visualizer_.publishCurrentSetpoint(toTwist(result.linear_velocity_wp, result.angular_velocity_wp),
                                     result.waypoint_type, newest_position_);
  visualizer_.publishOfftrackPoints(closest_pt, deg60_pt);

  auto obst_free_path = std::make_unique<mavros_msgs::msg::Trajectory>();
  transformToTrajectory(*obst_free_path, toPoseStamped(result.position_wp, result.orientation_wp),
                        toTwist(result.linear_velocity_wp, result.angular_velocity_wp));

  auto pos_setpoint = std::make_unique<geometry_msgs::msg::PoseStamped>(toPoseStamped(result.position_wp, result.orientation_wp));
  pos_setpoint->header.stamp = this->now();
  pos_setpoint->header.frame_id = frame_id_; // Ensure frame_id is set

  if(mavros_pos_setpoint_pub_) mavros_pos_setpoint_pub_->publish(*pos_setpoint);
  if(mavros_obstacle_free_path_pub_) mavros_obstacle_free_path_pub_->publish(std::move(*obst_free_path)); // std::move since unique_ptr content not needed
}


void LocalPlannerNodelet::clickedPointCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
  printPointInfo(msg->point.x, msg->point.y, msg->point.z);
}

void LocalPlannerNodelet::clickedGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> guard(running_mutex_);
  new_goal_ = true;
  prev_goal_position_ = goal_position_;
  goal_position_ = toEigen(msg->pose.position);
  if (local_planner_) {
    goal_position_.z() = local_planner_->getGoal().z(); // Maintain configured Z or last Z
  }
  RCLCPP_INFO(this->get_logger(), "New goal from RVIZ: [%.2f, %.2f, %.2f]", goal_position_.x(), goal_position_.y(), goal_position_.z());
}

void LocalPlannerNodelet::updateGoalCallback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> guard(running_mutex_);
  if (accept_goal_input_topic_ && !msg->markers.empty()) {
    prev_goal_position_ = goal_position_;
    goal_position_ = toEigen(msg->markers[0].pose.position);
    new_goal_ = true;
    RCLCPP_INFO(this->get_logger(), "New goal from topic: [%.2f, %.2f, %.2f]", goal_position_.x(), goal_position_.y(), goal_position_.z());
  }
}

void LocalPlannerNodelet::fcuInputGoalCallback(const mavros_msgs::msg::Trajectory::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> guard(running_mutex_);
  bool update = ((avoidance::toEigen(msg->point_2.position) - avoidance::toEigen(goal_mission_item_msg_.pose.position)).norm() > 0.01) ||
                !std::isfinite(goal_position_(0)) || !std::isfinite(goal_position_(1));
  if (!msg->point_valid.empty() && msg->point_valid[0] && update) {
    new_goal_ = true;
    prev_goal_position_ = goal_position_;
    goal_position_ = toEigen(msg->point_1.position);
    desired_velocity_ = toEigen(msg->point_1.velocity);
    if (!msg->command.empty()){
        is_land_waypoint_ = (msg->command[0] == static_cast<uint16_t>(MavCommand::MAV_CMD_NAV_LAND));
        is_takeoff_waypoint_ = (msg->command[0] == static_cast<uint16_t>(MavCommand::MAV_CMD_NAV_TAKEOFF));
    }
  }
  if (msg->point_valid.size() > 1 && msg->point_valid[1]) {
    goal_mission_item_msg_.pose.position = msg->point_2.position; // geometry_msgs::msg::Point
    if (msg->command.size() > 1 && msg->command[1] == UINT16_MAX) { // Check size before access
      goal_position_ = toEigen(msg->point_2.position);
      desired_velocity_ << NAN, NAN, NAN;
    }
    desired_yaw_setpoint_ = msg->point_2.yaw;
    desired_yaw_speed_setpoint_ = msg->point_2.yaw_rate;
  }
}

void LocalPlannerNodelet::distanceSensorCallback(const mavros_msgs::msg::Altitude::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> guard(running_mutex_);
  if (!std::isnan(msg->bottom_clearance)) {
    ground_distance_msg_ = *msg;
  }
}

void LocalPlannerNodelet::transformBufferThread() {
  while (rclcpp::ok() && !should_exit_) {
    {
      std::lock_guard<std::mutex> guard(buffered_transforms_mutex_);
      for (auto const& frame_pair : buffered_transforms_) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
          if (tf_buffer_->canTransform(frame_pair.second, frame_pair.first, tf2::TimePointZero)) {
            transform_stamped = tf_buffer_->lookupTransform(frame_pair.second, frame_pair.first, tf2::TimePointZero);
            custom_tf_buffer_.insertTransform(frame_pair.first, frame_pair.second, transform_stamped);
          }
        } catch (const tf2::TransformException& ex) {
          RCLCPP_ERROR(this->get_logger(), "TF Exception in transformBufferThread: %s", ex.what());
        }
      }
      tf_buffer_cv_.notify_all();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void LocalPlannerNodelet::printPointInfo(double x, double y, double z) {
  if(!local_planner_) return;
  Eigen::Vector3f drone_pos = local_planner_->getPosition();
  PolarPoint p_pol = cartesianToPolarHistogram(Eigen::Vector3f(x,y,z), drone_pos);

  RCLCPP_INFO(this->get_logger(),"----- Point: %f %f %f -----", x, y, z);
  RCLCPP_INFO(this->get_logger(),"Elevation %f Azimuth %f (Histogram convention)", p_pol.e, p_pol.z);
  RCLCPP_INFO(this->get_logger(),"--------------------------------------------");
}

void LocalPlannerNodelet::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, int index) {
  if (index >= cameras_.size()) return;
  std::lock_guard<std::mutex> lck(*(cameras_[index].camera_mutex_));

  rclcpp::Time last_cloud_received_time(cameras_[index].untransformed_cloud_.header.stamp.sec,
                                      cameras_[index].untransformed_cloud_.header.stamp.nanosec,
                                      this->get_clock()->get_clock_type());
  rclcpp::Time current_msg_time(msg->header.stamp.sec,
                                msg->header.stamp.nanosec,
                                this->get_clock()->get_clock_type());


  if (cameras_[index].received_ && (current_msg_time - last_cloud_received_time) < rclcpp::Duration::from_seconds(0.3)) {
    return;
  }

  pcl::fromROSMsg(*msg, cameras_[index].untransformed_cloud_);
  cameras_[index].untransformed_cloud_.header.stamp = pcl_conversions::toPCL(current_msg_time); // Store as PCL time
  cameras_[index].received_ = true;
  if (cameras_[index].camera_cv_) cameras_[index].camera_cv_->notify_all();

  if (!cameras_[index].transform_registered_) {
    std::lock_guard<std::mutex> tf_list_guard(buffered_transforms_mutex_);
    std::pair<std::string, std::string> transform_frames_local, transform_frames_fcu;
    transform_frames_local.first = msg->header.frame_id;
    transform_frames_local.second = "local_origin"; // Target frame for TF
    buffered_transforms_.push_back(transform_frames_local);
    transform_frames_fcu.first = msg->header.frame_id;
    transform_frames_fcu.second = "fcu"; // Target frame for TF
    buffered_transforms_.push_back(transform_frames_fcu);
    cameras_[index].transform_registered_ = true;
  }
}


void LocalPlannerNodelet::publishLaserScan() const {
  if (!local_planner_ || !(local_planner_->px4_.param_cp_dist < 0)) { // Check if planner exists and condition
    sensor_msgs::msg::LaserScan distance_data_to_fcu;
    local_planner_->getObstacleDistanceData(distance_data_to_fcu); // This method fills the message
    distance_data_to_fcu.header.stamp = this->now(); // Set stamp before publishing

    if (distance_data_to_fcu.angle_increment > 0.f && mavros_obstacle_distance_pub_) {
      mavros_obstacle_distance_pub_->publish(distance_data_to_fcu);
    }
  }
}

void LocalPlannerNodelet::threadFunction() {
  while (rclcpp::ok() && !should_exit_) {
    rclcpp::Time thread_start_time = this->now();

    size_t num_cameras = cameras_.size();
    if (num_cameras == 0 || numTransformedClouds() < num_cameras) {
         std::unique_lock<std::mutex> lock(transformed_cloud_mutex_);
         if (transformed_cloud_cv_.wait_for(lock, std::chrono::milliseconds(500)) == std::cv_status::timeout && !should_exit_ ){
             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Timeout waiting for transformed clouds (%zu/%zu available)", numTransformedClouds(), num_cameras);
             continue;
         }
    }
    if (should_exit_) break;

    {
      std::lock_guard<std::mutex> guard(running_mutex_); // Protects access to shared data
      updatePlannerInfo();
      if (local_planner_) local_planner_->runPlanner();

      if (local_planner_ && wp_generator_) { // Ensure pointers are valid
        visualizer_.visualizePlannerData(*(local_planner_.get()), newest_waypoint_position_,
                                         newest_adapted_waypoint_position_, newest_position_, newest_orientation_);
      }
      publishLaserScan();

      std::lock_guard<std::mutex> waypoints_guard(waypoints_mutex_); // Protects waypoints & planner_info in wp_generator
      if (local_planner_ && wp_generator_) {
        wp_generator_->setPlannerInfo(local_planner_->getAvoidanceOutput());
      }
      last_wp_time_ = this->now();
    }

    if (should_exit_) break;

    rclcpp::Duration loop_time = this->now() - thread_start_time;
    rclcpp::Duration required_delay(std::chrono::duration<double>(spin_dt_)); // spin_dt_ is double seconds
    if (loop_time < required_delay) {
      rclcpp::sleep_for((required_delay - loop_time).to_chrono<std::chrono::nanoseconds>());
    }
  }
}

void LocalPlannerNodelet::checkFailsafe(rclcpp::Duration since_last_cloud, rclcpp::Duration since_start, bool& hover_flag) {
    // If using avoidance_node_ptr_:
    // if(avoidance_node_ptr_) avoidance_node_ptr_->checkFailsafe(since_last_cloud, since_start, hover_flag);
    // else: implement basic failsafe or rely on external monitoring.
    // For now, replicating some of the logic from avoidance_node if it's not used.
    if (!local_planner_) return;

    if (since_last_cloud > rclcpp::Duration::from_seconds(local_planner_->timeout_termination_) &&
        since_start > rclcpp::Duration::from_seconds(local_planner_->timeout_termination_)) {
        setSystemStatus(MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
        RCLCPP_WARN(get_logger(), "\033[1;33m Planner abort: missing required data \n \033[0m");
    } else {
        if (since_last_cloud > rclcpp::Duration::from_seconds(local_planner_->timeout_critical_) &&
            since_start > rclcpp::Duration::from_seconds(local_planner_->timeout_startup_)) {
            if (position_received_) {
                hover_flag = true; // Modifying the passed-in hover_flag
                setSystemStatus(MAV_STATE::MAV_STATE_CRITICAL);
            } else {
                RCLCPP_WARN(get_logger(), "\033[1;33m Pointcloud timeout: No position received, no WP to output.... \n \033[0m");
            }
        } else {
            if (!hover_flag) setSystemStatus(MAV_STATE::MAV_STATE_ACTIVE);
        }
    }
}


void LocalPlannerNodelet::pointCloudTransformThread(int index) {
  while (rclcpp::ok() && !should_exit_) {
    bool should_wait_for_cloud = true;
    bool should_wait_for_transform = false;

    { // Scope for camera_mutex_
      std::unique_lock<std::mutex> camera_lock(*(cameras_[index].camera_mutex_));
      if (cameras_[index].camera_cv_->wait_for(camera_lock, std::chrono::milliseconds(500),
                                              [&]{ return cameras_[index].received_ || should_exit_; })) {
        if (should_exit_) break;
        should_wait_for_cloud = false; // Cloud received
      } else { // Timeout
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Timeout waiting for pointcloud on topic: %s", cameras_[index].topic_.c_str());
        continue; // Continue to check should_exit_ and rclcpp::ok()
      }
    } // camera_mutex_ released

    if (should_wait_for_cloud) continue;


    geometry_msgs::msg::TransformStamped transform_local, transform_fcu;
    rclcpp::Time cloud_stamp(cameras_[index].untransformed_cloud_.header.stamp.sec,
                             cameras_[index].untransformed_cloud_.header.stamp.nanosec,
                             this->get_clock()->get_clock_type());

    if (custom_tf_buffer_.getTransform(cameras_[index].untransformed_cloud_.header.frame_id, "local_origin", cloud_stamp, transform_local) &&
        custom_tf_buffer_.getTransform(cameras_[index].untransformed_cloud_.header.frame_id, "fcu", cloud_stamp, transform_fcu)) {

      std::lock_guard<std::mutex> camera_lock(*(cameras_[index].camera_mutex_)); // Lock again for PCL data access

      pcl::PointCloud<pcl::PointXYZ> cloud_in = cameras_[index].untransformed_cloud_; // Make a copy for processing
      pcl::PointCloud<pcl::PointXYZ> maxima = removeNaNAndGetMaxima(cloud_in); // operates in-place on cloud_in

      Eigen::Isometry3d eigen_transform_fcu = tf2::transformToEigen(transform_fcu);
      pcl::transformPointCloud(maxima, maxima, eigen_transform_fcu.cast<float>());
      updateFOVFromMaxima(cameras_[index].fov_fcu_frame_, maxima);

      Eigen::Isometry3d eigen_transform_local = tf2::transformToEigen(transform_local);
      pcl::transformPointCloud(cameras_[index].untransformed_cloud_, cameras_[index].transformed_cloud_, eigen_transform_local.cast<float>());

      cameras_[index].transformed_cloud_.header.frame_id = "local_origin";
      // Stamp is already in PCL format (microseconds)
      // cameras_[index].transformed_cloud_.header.stamp = cameras_[index].untransformed_cloud_.header.stamp;
      pcl_conversions::toPCL(cloud_stamp, cameras_[index].transformed_cloud_.header.stamp);


      cameras_[index].transformed_ = true;
      cameras_[index].received_ = false; // Ready for next cloud

      std::lock_guard<std::mutex> transformed_lock(transformed_cloud_mutex_);
      transformed_cloud_cv_.notify_all();

    } else {
      should_wait_for_transform = true;
    }

    if (should_wait_for_transform) {
      std::unique_lock<std::mutex> lck_tf(buffered_transforms_mutex_);
      tf_buffer_cv_.wait_for(lck_tf, std::chrono::milliseconds(100)); // Wait for new transforms from main TF thread
    }
  }
}


// Definition for cameraInfoCallback - if it's used. The header does not show it being subscribed.
// void LocalPlannerNodelet::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, int index) {
//   (void)msg; (void)index; // Mark as unused for now
//   // Logic to process camera info if needed
// }


}  // namespace avoidance

RCLCPP_COMPONENTS_REGISTER_NODE(avoidance::LocalPlannerNodelet)
