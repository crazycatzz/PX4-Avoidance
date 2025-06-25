#include "safe_landing_planner/waypoint_generator_node.hpp" // Updated header
#include "safe_landing_planner/safe_landing_planner.hpp"   // For MavCommand enum, if used from here (was in original includes)
#include "avoidance/common.h" // For toEigen, nextYaw etc. (already updated)

#include <tf2/utils.h> // For tf2::getYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For tf2::fromMsg, tf2::toMsg if needed for quaternion

#include <chrono>
#include <functional>

namespace avoidance {

// Assuming nan_setpoint and toString(SLPState) are still relevant and defined (possibly moved to common or waypoint_generator.hpp)
// const Eigen::Vector3f nan_setpoint = Eigen::Vector3f(NAN, NAN, NAN); // Defined in waypoint_generator.hpp now

WaypointGeneratorNode::WaypointGeneratorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("waypoint_generator_node", options),
      spin_dt_(0.1) // Default, will be updated by parameters
{
  RCLCPP_INFO(this->get_logger(), "Initializing WaypointGeneratorNode (ROS2)...");

  // Initialize parameters and ROS interfaces
  this->readParamsAndInitInterfaces();

  waypointGenerator_.publishTrajectorySetpoints_ =
    [this](const Eigen::Vector3f &pos_sp, const Eigen::Vector3f &vel_sp, float yaw_sp, float yaw_speed_sp) {
    this->publishTrajectorySetpoints(pos_sp, vel_sp, yaw_sp, yaw_speed_sp);
  };

  grid_received_ = false;
  goal_visualization_ = Eigen::Vector3f::Zero();

  RCLCPP_INFO(this->get_logger(), "WaypointGeneratorNode initialized.");
}

void WaypointGeneratorNode::readParamsAndInitInterfaces() {
  // Declare parameters (from WaypointGeneratorNodeConfig and nh_private_.param calls)
  this->declare_parameter<double>("spin_dt", 0.1);
  // Parameters for WaypointGenerator algorithm (were in WaypointGeneratorNodeConfig)
  this->declare_parameter<double>("beta", 0.9);
  this->declare_parameter<double>("can_land_thr", 0.4);
  this->declare_parameter<double>("loiter_height", 4.0);
  this->declare_parameter<int>("smoothing_land_cell", 6);
  this->declare_parameter<double>("vertical_range_error", 1.0);
  this->declare_parameter<double>("spiral_width", 2.0);
  // Add any other parameters that were in the config if they are used by this node or passed to waypointGenerator_

  // Get initial values & set up callback
  this->parametersCallback(this->get_parameters(this->list_parameters({}, 0).names));
  auto param_cb = std::bind(&WaypointGeneratorNode::parametersCallback, this, std::placeholders::_1);
  this->add_on_set_parameters_callback(param_cb);

  // Initialize ROS interfaces
  rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
  rclcpp::QoS latching_qos(rclcpp::KeepLast(1));
  latching_qos.transient_local();

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "mavros/local_position/pose", qos_profile, std::bind(&WaypointGeneratorNode::positionCallback, this, std::placeholders::_1));
  trajectory_sub_ = this->create_subscription<mavros_msgs::msg::Trajectory>(
      "mavros/trajectory/desired", qos_profile, std::bind(&WaypointGeneratorNode::trajectoryCallback, this, std::placeholders::_1));
  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "mavros/state", qos_profile, std::bind(&WaypointGeneratorNode::stateCallback, this, std::placeholders::_1));
  grid_sub_ = this->create_subscription<safe_landing_planner::msg::SLPGridMsg>(
      "grid_slp", qos_profile, std::bind(&WaypointGeneratorNode::gridCallback, this, std::placeholders::_1));
  // pos_index_sub_ was not clearly typed or used in the original snippet, omitting for now.

  trajectory_pub_ = this->create_publisher<mavros_msgs::msg::Trajectory>("mavros/trajectory/generated", qos_profile);
  land_hysteresis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/land_hysteresis", latching_qos);
  marker_goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("~/goal_position", latching_qos);

  // Initialize Timer
  cmdloop_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(spin_dt_), // spin_dt_ is set by parametersCallback
      std::bind(&WaypointGeneratorNode::cmdLoopCallback, this));
}


rcl_interfaces::msg::SetParametersResult WaypointGeneratorNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &param : parameters) {
    const std::string &name = param.get_name();
    RCLCPP_DEBUG(this->get_logger(), "WGN Updating parameter: %s", name.c_str());

    if (name == "spin_dt") {
        spin_dt_ = param.as_double();
        // Note: If timer period needs to change, timer must be canceled and recreated.
        // This simple update won't change existing timer's period.
        RCLCPP_WARN(this->get_logger(), "spin_dt changed, but timer period update isn't implemented yet.");
    } else if (name == "beta") waypointGenerator_.beta_ = static_cast<float>(param.as_double());
    else if (name == "can_land_thr") waypointGenerator_.can_land_thr_ = static_cast<float>(param.as_double());
    else if (name == "loiter_height") waypointGenerator_.loiter_height_ = static_cast<float>(param.as_double());
    else if (name == "smoothing_land_cell") {
        int new_smoothing_size = param.as_int();
        if (waypointGenerator_.smoothing_land_cell_ != new_smoothing_size) {
            waypointGenerator_.smoothing_land_cell_ = new_smoothing_size;
            waypointGenerator_.update_smoothing_size_ = true;
        }
    }
    else if (name == "vertical_range_error") waypointGenerator_.vertical_range_error_ = static_cast<float>(param.as_double());
    else if (name == "spiral_width") waypointGenerator_.spiral_width_ = static_cast<float>(param.as_double());
    else {
        RCLCPP_WARN(this->get_logger(), "WGN Unknown parameter: %s", name.c_str());
    }
  }
  return result;
}

void WaypointGeneratorNode::cmdLoopCallback() {
  if (!grid_received_ && rclcpp::ok()) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for grid_slp message...");
    return;
  }

  waypointGenerator_.calculateWaypoint();
  landingAreaVisualization();
  goalVisualization();
  grid_received_ = false;
}


void WaypointGeneratorNode::positionCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  waypointGenerator_.position_ = avoidance::toEigen(msg->pose.position);
  waypointGenerator_.yaw_ = static_cast<float>(tf2::getYaw(msg->pose.orientation)); // Use tf2::getYaw
  RCLCPP_DEBUG(this->get_logger(), "[WGN] Current position %f %f %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void WaypointGeneratorNode::trajectoryCallback(const mavros_msgs::msg::Trajectory::ConstSharedPtr msg) {
  bool update = ((avoidance::toEigen(msg->point_2.position) - goal_visualization_).norm() > 0.01) ||
                waypointGenerator_.goal_.topRows<2>().array().hasNaN();

  if (!msg->point_valid.empty() && msg->point_valid[0] && update) {
    waypointGenerator_.goal_ = avoidance::toEigen(msg->point_1.position);
    waypointGenerator_.velocity_setpoint_ = avoidance::toEigen(msg->point_1.velocity);
    if(!msg->command.empty()){
        waypointGenerator_.is_land_waypoint_ = (msg->command[0] == static_cast<uint16_t>(MavCommand::MAV_CMD_NAV_LAND));
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "\033[1;33m [WGN] Set New goal from FCU " << waypointGenerator_.goal_.transpose()
                                                              << " - nan nan nan \033[0m");
  }
  if (msg->point_valid.size() > 1 && msg->point_valid[1]) {
    goal_visualization_ = avoidance::toEigen(msg->point_2.position);
    waypointGenerator_.yaw_setpoint_ = msg->point_2.yaw;
    waypointGenerator_.yaw_speed_setpoint_ = msg->point_2.yaw_rate;
  }
}

void WaypointGeneratorNode::stateCallback(const mavros_msgs::msg::State::ConstSharedPtr msg) {
  std::string current_mode = msg->mode;
  if (current_mode == "AUTO.LAND") {
    waypointGenerator_.is_land_waypoint_ = true;
  } else if (current_mode != "AUTO.MISSION") { // In mission mode, is_land_waypoint_ is set by trajectoryCallback
    waypointGenerator_.is_land_waypoint_ = false;
    waypointGenerator_.trigger_reset_ = true;
  }

  if (!msg->armed) { // If disarmed
    waypointGenerator_.is_land_waypoint_ = false;
    waypointGenerator_.trigger_reset_ = true;
  }
}

void WaypointGeneratorNode::gridCallback(const safe_landing_planner::msg::SLPGridMsg::ConstSharedPtr msg) {
  waypointGenerator_.grid_slp_seq_ = msg->header.seq; // Assuming seq is int or compatible
  if (waypointGenerator_.grid_slp_.getGridSize() != msg->grid_size ||
      waypointGenerator_.grid_slp_.getCellSize() != msg.cell_size) {
    waypointGenerator_.grid_slp_.resize(msg->grid_size, msg->cell_size);
  }

  // Assuming msg.mean.data is float[] and layout is row-major
  // Also assuming SLPGridMsg structure matches this access pattern
  size_t rows = msg->mean.layout.dim[0].size;
  size_t cols = msg->mean.layout.dim[1].size;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      waypointGenerator_.grid_slp_.mean_(i, j) = msg->mean.data[msg->mean.layout.dim[1].stride * i + j];
      waypointGenerator_.grid_slp_.land_(i, j) = msg->land.data[msg->land.layout.dim[1].stride * i + j];
    }
  }

  waypointGenerator_.pos_index_.x() = static_cast<int>(msg->curr_pos_index.x);
  waypointGenerator_.pos_index_.y() = static_cast<int>(msg->curr_pos_index.y);

  waypointGenerator_.grid_slp_.setFilterLimits(waypointGenerator_.position_);
  grid_received_ = true;
}

void WaypointGeneratorNode::publishTrajectorySetpoints(const Eigen::Vector3f &pos_sp, const Eigen::Vector3f &vel_sp,
                                                       float yaw_sp, float yaw_speed_sp) {
  auto setpoint_msg = std::make_unique<mavros_msgs::msg::Trajectory>();
  setpoint_msg->header.stamp = this->now();
  setpoint_msg->header.frame_id = "local_origin"; // Should be a parameter or from state
  setpoint_msg->type = mavros_msgs::msg::Trajectory::MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS;

  setpoint_msg->point_1.position.x = pos_sp.x();
  setpoint_msg->point_1.position.y = pos_sp.y();
  setpoint_msg->point_1.position.z = pos_sp.z();
  setpoint_msg->point_1.velocity.x = vel_sp.x();
  setpoint_msg->point_1.velocity.y = vel_sp.y();
  setpoint_msg->point_1.velocity.z = vel_sp.z();
  setpoint_msg->point_1.acceleration_or_force.x = NAN;
  setpoint_msg->point_1.acceleration_or_force.y = NAN;
  setpoint_msg->point_1.acceleration_or_force.z = NAN;
  setpoint_msg->point_1.yaw = yaw_sp;
  setpoint_msg->point_1.yaw_rate = yaw_speed_sp;

  fillUnusedTrajectorySetpoints(setpoint_msg->point_2);
  fillUnusedTrajectorySetpoints(setpoint_msg->point_3);
  fillUnusedTrajectorySetpoints(setpoint_msg->point_4);
  fillUnusedTrajectorySetpoints(setpoint_msg->point_5);

  setpoint_msg->time_horizon = {NAN, NAN, NAN, NAN, NAN};

  bool xy_pos_sp_valid = std::isfinite(setpoint_msg->point_1.position.x) && std::isfinite(setpoint_msg->point_1.position.y);
  bool xy_vel_sp_valid = std::isfinite(setpoint_msg->point_1.velocity.x) && std::isfinite(setpoint_msg->point_1.velocity.y);

  if ((xy_pos_sp_valid || xy_vel_sp_valid) &&
      (std::isfinite(setpoint_msg->point_1.position.z) || std::isfinite(setpoint_msg->point_1.velocity.z))) { // Corrected logic
    setpoint_msg->point_valid = {true, false, false, false, false};
  } else {
    setpoint_msg->point_valid = {false, false, false, false, false};
  }

  if(trajectory_pub_) trajectory_pub_->publish(std::move(setpoint_msg));
}

void WaypointGeneratorNode::fillUnusedTrajectorySetpoints(mavros_msgs::msg::PositionTarget &point) {
  point.position.x = NAN; point.position.y = NAN; point.position.z = NAN;
  point.velocity.x = NAN; point.velocity.y = NAN; point.velocity.z = NAN;
  point.acceleration_or_force.x = NAN; point.acceleration_or_force.y = NAN; point.acceleration_or_force.z = NAN;
  point.yaw = NAN; point.yaw_rate = NAN;
}

void WaypointGeneratorNode::landingAreaVisualization() {
  if (!land_hysteresis_pub_ || !node_ptr_) return;
  visualization_msgs::msg::MarkerArray marker_array;

  float cell_size = waypointGenerator_.grid_slp_.getCellSize();
  visualization_msgs::msg::Marker cell;
  cell.header.frame_id = "local_origin"; // Should be a parameter
  cell.header.stamp = this->now();
  cell.id = 0;
  cell.type = visualization_msgs::msg::Marker::CUBE;
  cell.action = visualization_msgs::msg::Marker::ADD;
  cell.pose.orientation.w = 1.0;
  cell.scale.x = cell_size; cell.scale.y = cell_size; cell.scale.z = 0.1f;
  cell.color.a = 0.5f; cell.color.b = 0.0f;

  Eigen::Vector2f grid_min, grid_max;
  waypointGenerator_.grid_slp_.getGridLimits(grid_min, grid_max);
  int offset = waypointGenerator_.grid_slp_.land_.rows() / 2;

  Eigen::MatrixXi result = waypointGenerator_.can_land_hysteresis_result_.cwiseProduct(waypointGenerator_.mask_);

  int slc = waypointGenerator_.smoothing_land_cell_;

  for (int k = offset - slc; k <= offset + slc; k++) {
    for (int l = offset - slc; l <= offset + slc; l++) {
      if (k < 0 || k >= waypointGenerator_.grid_slp_.land_.rows() || l < 0 || l >= waypointGenerator_.grid_slp_.land_.cols()) continue;
      cell.pose.position.x = grid_min.x() + (cell_size * k) + (cell_size / 2.f);
      cell.pose.position.y = grid_min.y() + (cell_size * l) + (cell_size / 2.f);
      cell.pose.position.z = waypointGenerator_.altitude_landing_area_percentile_ ; // Visualize at detected altitude
      if (result(k,l)) { // Assuming result is same dimensions as grid_slp_.land_
        cell.color.r = 0.0f; cell.color.g = 1.0f;
      } else {
        cell.color.r = 1.0f; cell.color.g = 0.0f;
      }
      marker_array.markers.push_back(cell);
      cell.id++;
    }
  }
  land_hysteresis_pub_->publish(marker_array);
}

void WaypointGeneratorNode::goalVisualization() {
  if (!marker_goal_pub_ || !node_ptr_) return;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "local_origin"; // Should be a parameter
  m.header.stamp = this->now();
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.5f; m.scale.y = 0.5f; m.scale.z = 0.5f;
  m.color.a = 1.0f; m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f;
  m.lifetime = rclcpp::Duration(0,0);
  m.id = 0; // Ensure unique if multiple goals shown, or use fixed ID for current goal
  m.pose.position = avoidance::toPoint(goal_visualization_); // Use global goal_visualization_ member
  marker_goal_pub_->publish(m);
}

}  // namespace avoidance

// Main function if this is to be run standalone
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<avoidance::WaypointGeneratorNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// RCLCPP_COMPONENTS_REGISTER_NODE(avoidance::WaypointGeneratorNode) // If it's a component
