#include "avoidance/avoidance_node.h"
#include <chrono> // For std::chrono::duration used in timers

namespace avoidance {

// Constructor updated to take rclcpp::NodeOptions and call base Node constructor
AvoidanceNode::AvoidanceNode(const rclcpp::NodeOptions & options)
    : Node("avoidance_node", options), cmdloop_dt_(0.1), statusloop_dt_(0.2) { // Node name set here
  position_received_ = true; // Assuming this means some initial position is available or not critical at start
  should_exit_ = false;

  timeout_termination_ = 15.0; // Using double for duration conversions
  timeout_critical_ = 0.5;
  timeout_startup_ = 5.0;

  mission_item_speed_ = NAN;
  param_cb_mutex_.reset(new std::mutex);

  // Call init within the constructor or ensure it's called after construction
  // For ROS2, member initialization (pubs, subs, etc.) is typically done in constructor body.
  // The init() method pattern can still be used for organization.
  // Let's assume init() will be called by the code that creates an instance of AvoidanceNode,
  // or we can call it at the end of this constructor. For now, keeping it separate.
}

AvoidanceNode::~AvoidanceNode() {
  should_exit_ = true; // Signal worker thread to exit
  if (worker_.joinable()) {
    worker_.join();
  }
}

void AvoidanceNode::init() {
  // Initialize publishers
  mavros_system_status_pub_ = this->create_publisher<mavros_msgs::msg::CompanionProcessStatus>(
      "mavros/companion_process/status", 1); // QoS: 1 for latching-like behavior if needed, or rclcpp::SystemDefaultsQoS()

  // Initialize subscribers
  // Using std::placeholders::_1 for binding the message argument
  px4_param_sub_ = this->create_subscription<mavros_msgs::msg::Param>(
      "mavros/param/param_value", rclcpp::SystemDefaultsQoS(), // Adjust QoS as needed
      std::bind(&AvoidanceNode::px4ParamsCallback, this, std::placeholders::_1));
  mission_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
      "mavros/mission/waypoints", rclcpp::SystemDefaultsQoS(), // Adjust QoS as needed
      std::bind(&AvoidanceNode::missionCallback, this, std::placeholders::_1));

  // Initialize service clients
  get_px4_param_client_ = this->create_client<mavros_msgs::srv::ParamGet>("mavros/param/get");

  // Initialize timers
  cmdloop_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(cmdloop_dt_),
      std::bind(&AvoidanceNode::cmdLoopCallback, this));
  statusloop_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(statusloop_dt_),
      std::bind(&AvoidanceNode::statusLoopCallback, this));

  setSystemStatus(MAV_STATE::MAV_STATE_BOOT);

  // Spinners are not used in ROS2; executors handle callbacks.
  // If specific threading is needed for callbacks, use Callback Groups and add to custom Executor.

  worker_ = std::thread(&AvoidanceNode::checkPx4Parameters, this);
}

// cmdLoopCallback signature changed in header (no TimerEvent)
void AvoidanceNode::cmdLoopCallback() {
    // TODO: Implement actual command loop logic here
}

// statusLoopCallback signature changed in header (no TimerEvent)
void AvoidanceNode::statusLoopCallback() { publishSystemStatus(); }

void AvoidanceNode::setSystemStatus(MAV_STATE state) { companion_state_ = state; }

MAV_STATE AvoidanceNode::getSystemStatus() { return companion_state_; }

// Publish companion process status
void AvoidanceNode::publishSystemStatus() {
  auto status_msg = std::make_unique<mavros_msgs::msg::CompanionProcessStatus>(); // Use std::make_unique for ROS2 messages
  status_msg->header.stamp = this->now(); // Use node's clock
  status_msg->component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  status_msg->state = static_cast<int32_t>(companion_state_); // Use static_cast for enum

  if (mavros_system_status_pub_) { // Check if publisher is valid
    mavros_system_status_pub_->publish(std::move(status_msg));
  }
}

// checkFailsafe signature uses rclcpp::Duration
void AvoidanceNode::checkFailsafe(rclcpp::Duration since_last_cloud, rclcpp::Duration since_start, bool& hover) {
  rclcpp::Duration timeout_termination = rclcpp::Duration::from_seconds(timeout_termination_);
  rclcpp::Duration timeout_critical_duration = rclcpp::Duration::from_seconds(timeout_critical_); // Renamed to avoid conflict
  rclcpp::Duration timeout_startup_duration = rclcpp::Duration::from_seconds(timeout_startup_);   // Renamed to avoid conflict

  if (since_last_cloud > timeout_termination && since_start > timeout_termination) {
    setSystemStatus(MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
    RCLCPP_WARN(this->get_logger(), "\033[1;33m Planner abort: missing required data \n \033[0m");
  } else {
    if (since_last_cloud > timeout_critical_duration && since_start > timeout_startup_duration) {
      if (position_received_) {
        hover = true;
        setSystemStatus(MAV_STATE::MAV_STATE_CRITICAL);
        // std::string not_received = ""; // This variable was unused
      } else {
        RCLCPP_WARN(this->get_logger(), "\033[1;33m Pointcloud timeout: No position received, no WP to output.... \n \033[0m");
      }
    } else {
      if (!hover) setSystemStatus(MAV_STATE::MAV_STATE_ACTIVE);
    }
  }
}

// px4ParamsCallback signature updated to use ConstSharedPtr
void AvoidanceNode::px4ParamsCallback(const mavros_msgs::msg::Param::ConstSharedPtr msg) {
  // collect all px4_ parameters needed for model based trajectory planning
  // when adding new parameter to the struct ModelParameters,
  // add new else if case with correct value type
  auto parse_param_f = [this, &msg](const std::string& name, float& val) -> bool { // Added this capture
    if (msg->param_id == name) {
      RCLCPP_INFO(this->get_logger(), "parameter %s is set from  %f to %f \n", name.c_str(), val, msg->value.real);
      val = msg->value.real;
      return true;
    }
    return false;
  };

  auto parse_param_i = [this, &msg](const std::string& name, int& val) -> bool { // Added this capture
    if (msg->param_id == name) {
      RCLCPP_INFO(this->get_logger(), "parameter %s is set from %i to %li \n", name.c_str(), val, msg->value.integer);
      val = static_cast<int>(msg->value.integer); // Explicit cast if msg->value.integer is int64_t
      return true;
    }
    return false;
  };

  // clang-format off
  std::lock_guard<std::mutex> lck(*(param_cb_mutex_));
  parse_param_f("MPC_ACC_DOWN_MAX", px4_.param_mpc_acc_down_max) ||
  parse_param_f("MPC_ACC_HOR", px4_.param_mpc_acc_hor) ||
  parse_param_f("MPC_ACC_UP_MAX", px4_.param_mpc_acc_up_max) ||
  parse_param_i("MPC_AUTO_MODE", px4_.param_mpc_auto_mode) ||
  parse_param_f("MPC_JERK_MIN", px4_.param_mpc_jerk_min) ||
  parse_param_f("MPC_JERK_MAX", px4_.param_mpc_jerk_max) ||
  parse_param_f("MPC_LAND_SPEED", px4_.param_mpc_land_speed) ||
  parse_param_f("MPC_TKO_SPEED", px4_.param_mpc_tko_speed) ||
  parse_param_f("MPC_XY_CRUISE", px4_.param_mpc_xy_cruise) ||
  parse_param_f("MPC_Z_VEL_MAX_DN", px4_.param_mpc_z_vel_max_dn) ||
  parse_param_f("MPC_Z_VEL_MAX_UP", px4_.param_mpc_z_vel_max_up) ||
  parse_param_f("CP_DIST", px4_.param_cp_dist) ||
  parse_param_f("NAV_ACC_RAD", px4_.param_nav_acc_rad) ||
  parse_param_f("MPC_YAWRAUTO_MAX", px4_.param_mpc_yawrauto_max);
  // clang-format on
}

void AvoidanceNode::checkPx4Parameters() {
  // Use a local variable for the client to ensure it's valid if the node is destroyed.
  auto client = get_px4_param_client_;

  // Lambda for requesting a parameter
  // Changed to return bool success and take client as arg
  auto request_param = [this](rclcpp::Client<mavros_msgs::srv::ParamGet>::SharedPtr param_client,
                               const std::string& name, float& val) -> bool {
    if (!param_client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Service %s not available", param_client->get_service_name());
      return false;
    }
    auto request = std::make_shared<mavros_msgs::srv::ParamGet::Request>();
    request->param_id = name;

    auto future_result = param_client->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, std::chrono::seconds(1)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto result = future_result.get();
      if (result->success) {
        val = result->value.real; // Assuming real is the correct field for float params
        RCLCPP_DEBUG(this->get_logger(), "Successfully got param %s: %f", name.c_str(), val);
        return true;
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to get param %s", name.c_str());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service param/get for %s", name.c_str());
    }
    return false;
  };

  // Lambda for int parameters (if any are needed in the future, or if MPC_AUTO_MODE was int)
  // auto request_param_int = ... (similar structure for integer types)


  while (rclcpp::ok() && !should_exit_) { // Check rclcpp::ok()
    bool is_param_not_initialized = true; // Assume not initialized
    { // Scope for lock guard
      std::lock_guard<std::mutex> lck(*(param_cb_mutex_));
      // Request all parameters. If any request fails, it will use the old value or NAN.
      request_param(client, "MPC_ACC_HOR", px4_.param_mpc_acc_hor);
      request_param(client, "MPC_ACC_DOWN_MAX", px4_.param_mpc_acc_down_max);
      request_param(client, "MPC_ACC_UP_MAX", px4_.param_mpc_acc_up_max);
      request_param(client, "MPC_XY_CRUISE", px4_.param_mpc_xy_cruise);
      request_param(client, "MPC_Z_VEL_MAX_DN", px4_.param_mpc_z_vel_max_dn);
      request_param(client, "MPC_Z_VEL_MAX_UP", px4_.param_mpc_z_vel_max_up);
      request_param(client, "CP_DIST", px4_.param_cp_dist);
      request_param(client, "MPC_LAND_SPEED", px4_.param_mpc_land_speed);
      request_param(client, "MPC_JERK_MAX", px4_.param_mpc_jerk_max); // MPC_JERK_MIN was also in header struct?
      request_param(client, "NAV_ACC_RAD", px4_.param_nav_acc_rad);
      request_param(client, "MPC_YAWRAUTO_MAX", px4_.param_mpc_yawrauto_max);

      // Integer example if MPC_AUTO_MODE was fetched via service (currently it's from subscription)
      // request_param_int(client, "MPC_AUTO_MODE", px4_.param_mpc_auto_mode);


      is_param_not_initialized =
          !std::isfinite(px4_.param_mpc_xy_cruise) || !std::isfinite(px4_.param_cp_dist) ||
          !std::isfinite(px4_.param_mpc_land_speed) || !std::isfinite(px4_.param_nav_acc_rad) ||
          !std::isfinite(px4_.param_mpc_acc_hor) || !std::isfinite(px4_.param_mpc_jerk_max) ||
          !std::isfinite(px4_.param_mpc_acc_down_max) || !std::isfinite(px4_.param_mpc_acc_up_max) ||
          !std::isfinite(px4_.param_mpc_z_vel_max_dn) || !std::isfinite(px4_.param_mpc_z_vel_max_up) ||
          !std::isfinite(px4_.param_mpc_yawrauto_max) || (px4_.param_mpc_auto_mode == -1); // Check auto_mode too
    }

    if (is_param_not_initialized) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Some PX4 parameters are not yet initialized, retrying in 5s.");
      std::this_thread::sleep_for(std::chrono::seconds(5));
    } else {
      RCLCPP_DEBUG(this->get_logger(), "All PX4 parameters successfully checked/updated.");
      std::this_thread::sleep_for(std::chrono::seconds(30));
    }
  }
}

// missionCallback signature updated to use ConstSharedPtr
void AvoidanceNode::missionCallback(const mavros_msgs::msg::WaypointList::ConstSharedPtr msg) {
  for (int index = 0; index < static_cast<int>(msg->waypoints.size()); index++) { // Use msg->
    if (msg->waypoints[index].is_current) {
      for (int i = index; i >= 0; i--) {
        if (msg->waypoints[i].command == static_cast<int>(MavCommand::MAV_CMD_DO_CHANGE_SPEED) &&
            (msg->waypoints[i].param1 - 1.0f) < FLT_MIN && msg->waypoints[i].param2 > 0.0f) {
          // 1MAV_CMD_DO_CHANGE_SPEED, speed type: ground speed, speed valid
          mission_item_speed_ = msg->waypoints[i].param2;
          RCLCPP_DEBUG(this->get_logger(), "Mission item speed updated to: %f", mission_item_speed_);
          break;
        }
      }
      break;
    }
  }
}

ModelParameters AvoidanceNode::getPX4Parameters() const {
  std::lock_guard<std::mutex> lck(*(param_cb_mutex_));
  ModelParameters px4 = px4_;
  return px4;
}
}
