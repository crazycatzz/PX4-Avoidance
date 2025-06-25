#include "global_planner/global_planner_node.h" // Includes rclcpp.hpp, messages, etc.

#include <chrono>
#include <functional>
#include <tf2_eigen/tf2_eigen.hpp> // For TF <-> Eigen conversions
#include <pcl/common/transforms.h>  // For pcl::transformPointCloud

// octomap_msgs/conversions.h is included in global_planner_node.h
// pcl_conversions/pcl_conversions.h is included in global_planner_node.h

namespace global_planner {

GlobalPlannerNode::GlobalPlannerNode(const rclcpp::NodeOptions& options)
    : Node("global_planner_node", options),
      global_planner_()
      // avoidance_node_ptr_ and world_visualizer_ptr_ are initialized to nullptr by default
{
  RCLCPP_INFO(this->get_logger(), "Initializing GlobalPlannerNode (ROS2)...");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  this->readParamsAndInitInterfaces();

  last_wp_time_ = this->now();
  start_time_ = this->now();
  position_received_ = false;

  // Instantiation of other nodes: Commented out as this is not typical ROS2 node design.
  // These should ideally be separate nodes/components managed by a launch file or component container.
  /*
  rclcpp::NodeOptions sub_node_options;
  #ifndef DISABLE_SIMULATION
  world_visualizer_ptr_ = std::make_shared<avoidance::WorldVisualizer>(sub_node_options, "/global_planner_viz_from_gp_node");
  #endif
  avoidance_node_ptr_ = std::make_shared<avoidance::AvoidanceNode>(sub_node_options);
  // If these nodes are created, they need to be added to an executor.
  */

  RCLCPP_INFO(this->get_logger(), "GlobalPlannerNode constructor finished, ROS interfaces initialized.");
}

GlobalPlannerNode::~GlobalPlannerNode() {
  RCLCPP_INFO(this->get_logger(), "GlobalPlannerNode destroyed.");
}

void GlobalPlannerNode::readParamsAndInitInterfaces() {
  // Declare all parameters first (formerly nh_private_.param and dynamic_reconfigure)
  this->declare_parameter<double>("goal_max_dist_incline", 100.0);
  this->declare_parameter<double>("goal_max_dist_sl", 10.0);
  this->declare_parameter<double>("goal_max_vertical_diff_incline", 10.0);
  this->declare_parameter<double>("goal_max_vertical_diff_sl", 10.0);
  this->declare_parameter<double>("robot_radius", 0.5);
  this->declare_parameter<double>("max_path_dist_factor", 1.5);
  this->declare_parameter<double>("cmdloop_dt", 0.1);
  this->declare_parameter<double>("plannerloop_dt", 1.0);
  this->declare_parameter<double>("mapupdate_dt", 1.0);
  this->declare_parameter<std::string>("frame_id", "world");
  this->declare_parameter<std::string>("camera_frame_id", "world");
  this->declare_parameter<std::vector<std::string>>("pointcloud_topics", std::vector<std::string>());

  this->declare_parameter<int>("min_altitude", 1);
  this->declare_parameter<int>("max_altitude", 10);
  this->declare_parameter<double>("max_cell_risk", 0.2);
  this->declare_parameter<double>("smooth_factor", 10.0);
  this->declare_parameter<double>("vert_to_hor_cost", 1.0);
  this->declare_parameter<double>("risk_factor", 500.0);
  this->declare_parameter<double>("neighbor_risk_flow", 1.0);
  this->declare_parameter<double>("explore_penalty", 0.005);
  this->declare_parameter<double>("up_cost", 3.0);
  this->declare_parameter<double>("down_cost", 1.0);
  this->declare_parameter<double>("search_time", 0.5);
  this->declare_parameter<double>("min_overestimate_factor", 1.03);
  this->declare_parameter<double>("max_overestimate_factor", 2.0);
  this->declare_parameter<double>("risk_threshold_risk_based_speedup", 0.5);
  this->declare_parameter<double>("default_speed", 1.0);
  this->declare_parameter<double>("max_speed", 3.0);
  this->declare_parameter<int>("max_iterations", 2000);
  this->declare_parameter<bool>("goal_is_blocked", false);
  this->declare_parameter<bool>("current_cell_blocked", false);
  this->declare_parameter<bool>("goal_must_be_free", true);
  this->declare_parameter<bool>("use_current_yaw", true);
  this->declare_parameter<bool>("use_risk_heuristics", true);
  this->declare_parameter<bool>("use_speedup_heuristics", true);
  this->declare_parameter<bool>("use_risk_based_speedup", true);
  this->declare_parameter<std::string>("default_node_type", "SpeedNode");
  this->declare_parameter<double>("clicked_goal_altitude", 0.0);
  this->declare_parameter<double>("clicked_goal_radius", 1.0);
  this->declare_parameter<bool>("hover", false);
  this->declare_parameter<int>("simplify_iterations", 0);
  this->declare_parameter<double>("simplify_margin", 1.01);

  // Get all declared parameters and apply them initially
  // This will call parametersCallback once with all declared values
  this->parametersCallback(this->get_parameters(this->list_parameters({}, 0).names));

  // Setup parameter callback for future changes
  auto param_cb = std::bind(&GlobalPlannerNode::parametersCallback, this, std::placeholders::_1);
  this->add_on_set_parameters_callback(param_cb);

  // Initialize Publishers
  rclcpp::QoS qos_profile(rclcpp::KeepLast(10)); // Default QoS
  rclcpp::QoS latching_qos(rclcpp::KeepLast(1)); // For "latched" topics
  latching_qos.transient_local();


  global_temp_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/global_temp_path", qos_profile);
  smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/smooth_path", qos_profile);
  actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/actual_path", qos_profile);
  explored_cells_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/explored_cells", latching_qos);
  global_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/global_goal", latching_qos);
  global_temp_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/global_temp_goal", latching_qos);
  mavros_obstacle_free_path_pub_ = this->create_publisher<mavros_msgs::msg::Trajectory>("mavros/trajectory/generated", qos_profile);
  mavros_waypoint_publisher_ = this->create_publisher<mavros_msgs::msg::Trajectory>("mavros/trajectory/waypoints", qos_profile);
  current_waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/current_setpoint", latching_qos);
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/pointcloud", qos_profile);

  // Initialize Subscribers
  octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "octomap", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&GlobalPlannerNode::octomapFullCallback, this, std::placeholders::_1));
  octomap_full_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "octomap_full", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&GlobalPlannerNode::octomapFullCallback, this, std::placeholders::_1));
  ground_truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", qos_profile, std::bind(&GlobalPlannerNode::positionCallbackOdometry, this, std::placeholders::_1)); // Needs a new callback for Odometry
  velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "mavros/local_position/velocity_body", qos_profile, std::bind(&GlobalPlannerNode::velocityCallback, this, std::placeholders::_1));
  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", qos_profile, std::bind(&GlobalPlannerNode::clickedPointCallback, this, std::placeholders::_1));
  move_base_simple_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "move_base_simple/goal", qos_profile, std::bind(&GlobalPlannerNode::moveBaseSimpleCallback, this, std::placeholders::_1));
  fcu_input_sub_ = this->create_subscription<mavros_msgs::msg::Trajectory>(
      "path_from_fcu", qos_profile, std::bind(&GlobalPlannerNode::fcuInputGoalCallback, this, std::placeholders::_1));

  std::vector<std::string> camera_topics;
  this->get_parameter("pointcloud_topics", camera_topics); // Get already declared param
  initializeCameraSubscribers(camera_topics);

  // Initialize Timers (cmdloop_dt_ and plannerloop_dt_ are set by parametersCallback)
  cmdloop_timer_ = this->create_wall_timer(
     std::chrono::duration<double>(cmdloop_dt_), std::bind(&GlobalPlannerNode::cmdLoopCallback, this));
  plannerloop_timer_ = this->create_wall_timer(
     std::chrono::duration<double>(plannerloop_dt_), std::bind(&GlobalPlannerNode::plannerLoopCallback, this));
}

rcl_interfaces::msg::SetParametersResult GlobalPlannerNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  std::lock_guard<std::mutex> lock(mutex_); // Protect member access

  for (const auto &param : parameters) {
    const std::string &name = param.get_name();
    RCLCPP_DEBUG(this->get_logger(), "Updating parameter: %s", name.c_str());

    // GlobalPlanner class parameters
    if (name == "min_altitude") global_planner_.min_altitude_ = param.as_int();
    else if (name == "max_altitude") global_planner_.max_altitude_ = param.as_int();
    else if (name == "max_cell_risk") global_planner_.max_cell_risk_ = param.as_double();
    else if (name == "smooth_factor") global_planner_.smooth_factor_ = param.as_double();
    else if (name == "vert_to_hor_cost") global_planner_.vert_to_hor_cost_ = param.as_double();
    else if (name == "risk_factor") global_planner_.risk_factor_ = param.as_double();
    else if (name == "neighbor_risk_flow") global_planner_.neighbor_risk_flow_ = param.as_double();
    else if (name == "explore_penalty") global_planner_.explore_penalty_ = param.as_double();
    else if (name == "up_cost") global_planner_.up_cost_ = param.as_double();
    else if (name == "down_cost") global_planner_.down_cost_ = param.as_double();
    else if (name == "search_time") global_planner_.search_time_ = param.as_double();
    else if (name == "min_overestimate_factor") global_planner_.min_overestimate_factor_ = param.as_double();
    else if (name == "max_overestimate_factor") global_planner_.max_overestimate_factor_ = param.as_double();
    else if (name == "risk_threshold_risk_based_speedup") global_planner_.risk_threshold_risk_based_speedup_ = param.as_double();
    else if (name == "default_speed") global_planner_.default_speed_ = param.as_double();
    else if (name == "max_speed") global_planner_.max_speed_ = param.as_double();
    else if (name == "max_iterations") global_planner_.max_iterations_ = param.as_int();
    else if (name == "goal_is_blocked") global_planner_.goal_is_blocked_ = param.as_bool();
    else if (name == "current_cell_blocked") global_planner_.current_cell_blocked_ = param.as_bool();
    else if (name == "goal_must_be_free") global_planner_.goal_must_be_free_ = param.as_bool();
    else if (name == "use_current_yaw") global_planner_.use_current_yaw_ = param.as_bool();
    else if (name == "use_risk_heuristics") global_planner_.use_risk_heuristics_ = param.as_bool();
    else if (name == "use_speedup_heuristics") global_planner_.use_speedup_heuristics_ = param.as_bool();
    else if (name == "use_risk_based_speedup") global_planner_.use_risk_based_speedup_ = param.as_bool();
    else if (name == "default_node_type") global_planner_.default_node_type_ = param.as_string();
    // GlobalPlannerNode direct members
    else if (name == "clicked_goal_altitude") clicked_goal_alt_ = param.as_double();
    else if (name == "clicked_goal_radius") clicked_goal_radius_ = param.as_double();
    else if (name == "hover") hover_ = param.as_bool();
    else if (name == "simplify_iterations") simplify_iterations_ = param.as_int();
    else if (name == "simplify_margin") simplify_margin_ = param.as_double();
    else if (name == "cmdloop_dt") cmdloop_dt_ = param.as_double(); // Timer rates might need recreation if changed
    else if (name == "plannerloop_dt") plannerloop_dt_ = param.as_double();
    else if (name == "mapupdate_dt") mapupdate_dt_ = param.as_double();
    else if (name == "frame_id") { frame_id_ = param.as_string(); global_planner_.setFrame(frame_id_); }
    else if (name == "camera_frame_id") camera_frame_id_ = param.as_string();
    else if (name == "pointcloud_topics") {
        RCLCPP_WARN(this->get_logger(), "'pointcloud_topics' changed, re-initializing camera subscribers.");
        initializeCameraSubscribers(param.as_string_array());
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown parameter update: %s", name.c_str());
    }
  }
  if (result.successful) {
    global_planner_.calculateAccumulatedHeightPrior();
    RCLCPP_INFO(this->get_logger(), "Parameters reconfigured successfully.");
  }
  return result;
}

void GlobalPlannerNode::initializeCameraSubscribers(const std::vector<std::string>& camera_topics) {
  cameras_.clear(); // Clear existing subscribers if any
  cameras_.resize(camera_topics.size());
  for (size_t i = 0; i < camera_topics.size(); ++i) {
    auto callback = [this, topic_name = camera_topics[i]](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        this->depthCameraCallback(msg, topic_name);
    };
    cameras_[i].pointcloud_sub_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(camera_topics[i], rclcpp::SensorDataQoS(), callback);
    RCLCPP_INFO(this->get_logger(), "Subscribed to pointcloud topic: %s", camera_topics[i].c_str());
  }
}

void GlobalPlannerNode::receivePath(const nav_msgs::msg::Path::ConstSharedPtr msg) {
  std::vector<geometry_msgs::msg::PoseStamped> path;
  for (const auto& p : msg->poses) {
    path.push_back(p);
  }
  setCurrentPath(path);
  RCLCPP_INFO(this->get_logger(), "Received a path of length %zu", path.size());
}

void GlobalPlannerNode::setNewGoal(const GoalCell& goal) {
  std::lock_guard<std::mutex> lock(mutex_);
  global_planner_.setGoal(goal);
  publishGoal(goal);
}

void GlobalPlannerNode::popNextGoal() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (waypoints_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Waypoints empty");
    return;
  }
  GoalCell new_goal = waypoints_.front();
  waypoints_.erase(waypoints_.begin());
  setNewGoal(new_goal); // setNewGoal already locks, but this function is public
}

void GlobalPlannerNode::planPath() {
  std::lock_guard<std::mutex> lock(mutex_);
  bool success = global_planner_.getGlobalPath();
  if (success) {
    publishPath();
  }
}

void GlobalPlannerNode::setIntermediateGoal() {
  std::lock_guard<std::mutex> lock(mutex_);
  GoalCell new_goal = global_planner_.goal_pos_; // Start with current final goal
  if (global_planner_.curr_path_.size() > 2) {
    int intermediate_goal_idx = std::min(static_cast<int>(global_planner_.curr_path_.size() - 1), 10);
    new_goal = GoalCell(global_planner_.curr_path_[intermediate_goal_idx], clicked_goal_radius_, true);
    auto pose_msg = global_planner_.createPoseMsg(new_goal, global_planner_.curr_yaw_);
    pose_msg.header.stamp = this->now();
    global_temp_goal_pub_->publish(pose_msg);
  }
  setNewGoal(new_goal); // setNewGoal already locks
}

bool GlobalPlannerNode::isCloseToGoal() {
  std::lock_guard<std::mutex> lock(mutex_);
  return global_planner_.goal_pos_.withinPositionRadius(global_planner_.curr_pos_);
}

void GlobalPlannerNode::setCurrentPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Cell> path;
  for (const auto& p : poses) {
    path.push_back(Cell(p.pose.position));
  }
  global_planner_.setPath(path);
  publishPath(); // publishPath already locks if needed
}

void GlobalPlannerNode::velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  global_planner_.curr_vel_ = msg->twist.linear;
}

// Need a different callback for Odometry if ground_truth_sub_ is Odometry
void GlobalPlannerNode::positionCallbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->pose.pose; // Extract Pose from PoseWithCovariance

    if (!position_received_) {
        start_pos_ = pose_stamped.pose.position;
        global_planner_.setFrame(msg->header.frame_id);
    }
    last_pos_ = pose_stamped;
    global_planner_.setPose(pose_stamped);
    position_received_ = true;

    if (num_pos_msg_++ % 50 == 0) {
        RCLCPP_INFO(this->get_logger(), "Current position (from Odom): %2.2f, %2.2f, %2.2f",
                    pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z);
    }
    actual_path_.header = pose_stamped.header;
    actual_path_.poses.push_back(pose_stamped);
    actual_path_pub_->publish(actual_path_);
}


void GlobalPlannerNode::positionCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!position_received_) {
    start_pos_ = msg->pose.position;
    global_planner_.setFrame(msg->header.frame_id);
  }
  last_pos_ = *msg;
  global_planner_.setPose(*msg);
  position_received_ = true;
  if (num_pos_msg_++ % 50 == 0) {
    RCLCPP_INFO(this->get_logger(), "Current position: %2.2f, %2.2f, %2.2f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }
  actual_path_.header = msg->header;
  actual_path_.poses.push_back(*msg);
  actual_path_pub_->publish(actual_path_);
}

void GlobalPlannerNode::clickedPointCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  Cell new_goal_cell = Cell(msg->point.x, msg->point.y, clicked_goal_alt_);
  GoalCell new_goal(new_goal_cell, clicked_goal_radius_);
  RCLCPP_INFO(this->get_logger(), "New goal: (%2.2f, %2.2f, %2.2f) at altitude %2.2f", msg->point.x, msg->point.y, msg->point.z,
           clicked_goal_alt_);
  auto pose_msg = global_planner_.createPoseMsg(new_goal, 0.0);
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = msg->header.frame_id; // Use frame_id from clicked point
  last_clicked_points.push_back(pose_msg);
  waypoints_.push_back(new_goal);
}

void GlobalPlannerNode::moveBaseSimpleCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  Cell new_goal_cell = Cell(msg->pose.position);
  GoalCell new_goal(new_goal_cell, clicked_goal_radius_);
  RCLCPP_INFO(this->get_logger(), "New goal from move_base_simple: (%2.2f, %2.2f, %2.2f) at altitude %2.2f", msg->pose.position.x,
           msg->pose.position.y, msg->pose.position.z, clicked_goal_alt_); // clicked_goal_alt_ might not be what user wants here
  last_clicked_points.push_back(*msg);
  waypoints_.push_back(new_goal);
}

void GlobalPlannerNode::octomapFullCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (num_octomap_msg_++ % 10 == 0) {
    RCLCPP_INFO(this->get_logger(), "Received full octomap (seq: %u)", msg->header.stamp.sec); // Using sec as stand-in for seq
  }
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  if (tree) {
    global_planner_.updateFullOctomap(tree);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message to AbstractOcTree");
  }
}

void GlobalPlannerNode::depthCameraCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const std::string& topic_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*msg, pcl_cloud);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received pointcloud from %s with %zu points", topic_name.c_str(), pcl_cloud.points.size());

  if (!frame_id_.empty() && !msg->header.frame_id.empty() && tf_buffer_->_frameExists(msg->header.frame_id) && tf_buffer_->_frameExists(frame_id_)) {
      try {
          geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
              frame_id_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

          Eigen::Isometry3d eigen_transform = tf2::transformToEigen(transform_stamped);
          pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
          pcl::transformPointCloud(pcl_cloud, transformed_cloud, eigen_transform.cast<float>());

          if (global_planner_.octree_) {
              for (const auto& p : transformed_cloud) {
                  global_planner_.octree_->updateNode(p.x, p.y, p.z, true);
              }
          }
          global_planner_.risk_cache_.clear();
      } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "Could not transform point cloud from %s to %s: %s",
              msg->header.frame_id.c_str(), frame_id_.c_str(), ex.what());
      }
  } else {
      RCLCPP_WARN(this->get_logger(), "Frames %s or %s do not exist in TF buffer, or msg frame_id empty, for point cloud from %s.",
          msg->header.frame_id.c_str(), frame_id_.c_str(), topic_name.c_str());
  }
}

void GlobalPlannerNode::fcuInputGoalCallback(const mavros_msgs::msg::Trajectory::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg->point_valid.empty() && msg->point_valid[0] == true && (msg->point_valid.size() <= 1 || msg->point_valid[1] == false)) {
    Cell new_goal_cell(msg->point_1.position.x, msg->point_1.position.y, msg->point_1.position.z);
    GoalCell new_goal(new_goal_cell, clicked_goal_radius_);
    waypoints_.push_back(new_goal);
    RCLCPP_INFO(this->get_logger(), "Received new waypoints from FCU (%2.2f, %2.2f, %2.2f)", msg->point_1.position.x, msg->point_1.position.y,
             msg->point_1.position.z);
  }
}

void GlobalPlannerNode::cmdLoopCallback() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (waypoints_.empty()) {
    return;
  }

  GoalCell curr_goal = waypoints_.front(); // Make a copy
  bool close_to_goal = global_planner_.goal_pos_.withinPositionRadius(global_planner_.curr_pos_);

  if (close_to_goal) {
    if (curr_goal.is_temporary_) {
      RCLCPP_INFO(this->get_logger(), "Intermediate goal reached");
      // Revert to the main goal this temporary goal was for.
      // This logic assumes global_planner_.goal_pos_ still holds the "true" destination.
      setNewGoal(global_planner_.goal_pos_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal reached");
      popNextGoal(); // Gets next main waypoint, setNewGoal is called inside
    }
    last_wp_time_ = this->now();
  }
  publishSetpoint();
}

void GlobalPlannerNode::plannerLoopCallback() {
  // position_received_ check might be redundant if node only starts planning after position is known
  if (position_received_ && !waypoints_.empty() && !isCloseToGoal()) {
    planPath();
  }
}

void GlobalPlannerNode::publishGoal(const GoalCell& goal) {
  auto pose_msg = global_planner_.createPoseMsg(goal, global_planner_.curr_yaw_);
  pose_msg.header.stamp = this->now();
  global_goal_pub_->publish(pose_msg);
}

void GlobalPlannerNode::publishPath() {
  nav_msgs::msg::Path path_msg = global_planner_.getPathMsg();
  path_msg.header.stamp = this->now();
  smooth_path_pub_->publish(path_msg);

  // Publish path with risk (optional)
  // auto path_risk_msg = global_planner_.getPathWithRiskMsg();
  // path_risk_msg.header.stamp = this->now();
  // global_temp_path_pub_->publish(path_risk_msg);
}

void GlobalPlannerNode::publishSetpoint() {
  auto msg = std::make_unique<mavros_msgs::msg::Trajectory>();
  msg->header.stamp = this->now();
  msg->header.frame_id = frame_id_; // frame_id_ is a member set from params
  msg->type = mavros_msgs::msg::Trajectory::MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS;

  if (global_planner_.curr_path_.size() > 1) {
    Cell curr = global_planner_.curr_path_[0];
    Cell next = global_planner_.curr_path_[1];
    msg->point_1.position = curr.toPoint();
    msg->point_1.yaw = static_cast<float>(nextYaw(curr, next, global_planner_.curr_yaw_));
    msg->point_1.velocity.x = static_cast<float>(speed_ * std::cos(msg->point_1.yaw)); // speed_ is a member
    msg->point_1.velocity.y = static_cast<float>(speed_ * std::sin(msg->point_1.yaw));
    msg->point_1.velocity.z = 0.0f;
    msg->point_valid[0] = true;
    auto current_wp_pose = global_planner_.createPoseMsg(curr, msg->point_1.yaw);
    current_wp_pose.header.stamp = this->now();
    current_waypoint_publisher_->publish(current_wp_pose);
  } else if (global_planner_.curr_path_.size() == 1) {
    Cell curr = global_planner_.curr_path_[0];
    msg->point_1.position = curr.toPoint();
    msg->point_1.yaw = static_cast<float>(global_planner_.curr_yaw_);
    msg->point_valid[0] = true;
    auto current_wp_pose = global_planner_.createPoseMsg(curr, global_planner_.curr_yaw_);
    current_wp_pose.header.stamp = this->now();
    current_waypoint_publisher_->publish(current_wp_pose);
  } else {
    msg->point_valid[0] = false;
  }

  for (size_t i = 1; i < msg->point_valid.size(); ++i) { // Iterate up to point_valid size
    if (i < 5) msg->point_valid[i] = false; // Original loop was up to 5
  }
  mavros_obstacle_free_path_pub_->publish(std::move(msg));
}

void GlobalPlannerNode::printPointInfo(double x, double y, double z) {
  printPointStats(&global_planner_, x, y, z);
}

}  // namespace global_planner
