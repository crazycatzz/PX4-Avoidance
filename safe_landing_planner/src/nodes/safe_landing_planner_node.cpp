#include "safe_landing_planner/safe_landing_planner_node.hpp"
#include "avoidance/common.h" // For toEigen, getYawFromQuaternion etc. (already updated)
#include <tf2_eigen/tf2_eigen.hpp> // For TF <-> Eigen conversions
#include <pcl/common/transforms.h>  // For pcl::transformPointCloud
#include <sensor_msgs/image_encodings.hpp> // For image encodings if used by visualization directly

#include <chrono>
#include <functional>
#include <string>
#include <vector>

namespace avoidance {

// const Eigen::Vector3f nan_setpoint = Eigen::Vector3f(NAN, NAN, NAN); // Defined in WaypointGenerator, not needed here directly

SafeLandingPlannerNode::SafeLandingPlannerNode(const rclcpp::NodeOptions& options)
    : Node("safe_landing_planner_node", options),
      spin_dt_(0.1) // Default, will be updated by parameters
{
  RCLCPP_INFO(this->get_logger(), "Initializing SafeLandingPlannerNode (ROS2)...");

  safe_landing_planner_ = std::make_unique<SafeLandingPlanner>();

  // Initialize TF2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // cloud_msg_mutex_, transformed_cloud_mutex_, cloud_ready_cv_ are unique_ptrs, initialize them
  cloud_msg_mutex_ = std::make_unique<std::mutex>();
  transformed_cloud_mutex_ = std::make_unique<std::mutex>();
  cloud_ready_cv_ = std::make_unique<std::condition_variable>();

  // Parameter handling, ROS interface initialization
  this->readParamsAndInitInterfaces();

  visualizer_.initializePublishers(std::dynamic_pointer_cast<rclcpp::Node>(this->get_node_base_interface()));

#ifndef DISABLE_SIMULATION
  // world_visualizer_ptr_ instantiation commented out, similar to other nodes
  // rclcpp::NodeOptions viz_options;
  // world_visualizer_ptr_ = std::make_shared<avoidance::WorldVisualizer>(viz_options, this->get_name());
#endif

  start_time_ = this->now();
  last_algo_time_ = this->now();
  t_status_sent_ = this->now();
  position_received_ = false;
  cloud_transformed_ = false;

  // Start worker thread for point cloud transformation
  worker_ = std::thread(&SafeLandingPlannerNode::pointCloudTransformThread, this);

  RCLCPP_INFO(this->get_logger(), "SafeLandingPlannerNode initialized.");
}

SafeLandingPlannerNode::~SafeLandingPlannerNode() {
    RCLCPP_INFO(this->get_logger(), "Destroying SafeLandingPlannerNode...");
    should_exit_ = true;
    if (cloud_ready_cv_) cloud_ready_cv_->notify_all(); // Notify to allow thread to exit
    if (worker_.joinable()) {
        worker_.join();
    }
    RCLCPP_INFO(this->get_logger(), "SafeLandingPlannerNode threads joined.");
}


void SafeLandingPlannerNode::readParamsAndInitInterfaces() {
    // Declare parameters
    this->declare_parameter<std::string>("pointcloud_topic", "camera/depth/points");
    this->declare_parameter<bool>("play_rosbag", false);
    this->declare_parameter<double>("spin_dt", 0.1);

    // Parameters for SafeLandingPlanner algorithm (from SafeLandingPlannerNodeConfig)
    this->declare_parameter<double>("timeout_critical", 0.5);
    this->declare_parameter<double>("timeout_termination", 15.0);
    this->declare_parameter<double>("n_points_threshold", 1.0); // Was float n_points_thr_
    this->declare_parameter<double>("std_dev_threshold", 0.1);  // Was float std_dev_thr_
    this->declare_parameter<double>("grid_size", 10.0);
    this->declare_parameter<double>("cell_size", 1.0);
    this->declare_parameter<double>("mean_diff_thr", 0.3);
    this->declare_parameter<double>("alpha", 0.8);
    // this->declare_parameter<int>("n_lines_padding", 1); // n_lines_padding_ is set by smoothing_size
    this->declare_parameter<int>("max_n_mean_diff_cells", 2);
    this->declare_parameter<int>("smoothing_size", 1);
    this->declare_parameter<int>("min_n_land_cells", 9);

    // Get initial values & set up callback
    this->parametersCallback(this->get_parameters(this->list_parameters({}, 0).names));
    auto param_cb = std::bind(&SafeLandingPlannerNode::parametersCallback, this, std::placeholders::_1);
    this->add_on_set_parameters_callback(param_cb);

    // Initialize ROS interfaces
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    rclcpp::QoS latching_qos(rclcpp::KeepLast(1));
    latching_qos.transient_local();

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", qos_profile, std::bind(&SafeLandingPlannerNode::positionCallback, this, std::placeholders::_1));

    std::string camera_topic_str;
    this->get_parameter("pointcloud_topic", camera_topic_str);
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        camera_topic_str, rclcpp::SensorDataQoS(), std::bind(&SafeLandingPlannerNode::pointCloudCallback, this, std::placeholders::_1));

    mavros_system_status_pub_ = this->create_publisher<mavros_msgs::msg::CompanionProcessStatus>(
        "mavros/companion_process/status", latching_qos);
    grid_pub_ = this->create_publisher<safe_landing_planner::msg::SLPGridMsg>("~/grid_slp", latching_qos);

    if (safe_landing_planner_->play_rosbag_) {
        raw_grid_sub_ = this->create_subscription<safe_landing_planner::msg::SLPGridMsg>(
            "/raw_grid_slp", qos_profile, std::bind(&SafeLandingPlannerNode::rawGridCallback, this, std::placeholders::_1));
        if (pointcloud_sub_) { // If playing rosbag, we might not need the live pointcloud sub
             RCLCPP_INFO(this->get_logger(), "Playing from rosbag, pointcloud_sub shutdown is not direct in ROS2. Unsubscribe by resetting.");
             pointcloud_sub_.reset();
        }
    }

    cmdloop_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(spin_dt_), std::bind(&SafeLandingPlannerNode::cmdLoopCallback, this));
}


rcl_interfaces::msg::SetParametersResult SafeLandingPlannerNode::parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    std::lock_guard<std::mutex> guard(*cloud_msg_mutex_); // Protect access to shared data like safe_landing_planner_ members

    bool play_rosbag_changed = false;
    std::string new_pointcloud_topic = "";

    for (const auto &param : parameters) {
        const std::string &name = param.get_name();
        RCLCPP_DEBUG(this->get_logger(), "SLPN Updating parameter: %s", name.c_str());

        if (name == "pointcloud_topic") new_pointcloud_topic = param.as_string();
        else if (name == "play_rosbag") {
            if (safe_landing_planner_->play_rosbag_ != param.as_bool()) play_rosbag_changed = true;
            safe_landing_planner_->play_rosbag_ = param.as_bool();
        }
        else if (name == "spin_dt") spin_dt_ = param.as_double(); // Timer period change needs timer recreation
        // Pass relevant parameters to SafeLandingPlanner algorithm class
        else if (name == "timeout_critical" || name == "timeout_termination" || name == "n_points_threshold" ||
                 name == "std_dev_threshold" || name == "grid_size" || name == "cell_size" ||
                 name == "mean_diff_thr" || name == "alpha" || name == "max_n_mean_diff_cells" ||
                 name == "smoothing_size" || name == "min_n_land_cells") {
            // Collect all params for SafeLandingPlanner and call its update method once
        } else {
             RCLCPP_WARN(this->get_logger(), "SLPN Unknown parameter: %s", name.c_str());
        }
    }

    // Update SafeLandingPlanner algorithm parameters
    if (safe_landing_planner_) {
        safe_landing_planner_->updateSLPParams(
            this->get_parameter("n_points_threshold").as_double(), // was float n_points_thr_
            this->get_parameter("std_dev_threshold").as_double(),  // was float std_dev_thr_
            this->get_parameter("grid_size").as_double(),
            this->get_parameter("cell_size").as_double(),
            this->get_parameter("mean_diff_thr").as_double(),
            this->get_parameter("alpha").as_double(),
            this->get_parameter("smoothing_size").as_int(), // n_lines_padding in SLP is set by smoothing_size
            this->get_parameter("max_n_mean_diff_cells").as_int(),
            this->get_parameter("smoothing_size").as_int(),
            this->get_parameter("min_n_land_cells").as_int(),
            this->get_parameter("timeout_critical").as_double(),
            this->get_parameter("timeout_termination").as_double(),
            this->get_parameter("play_rosbag").as_bool()
        );
    }

    // Handle pointcloud topic change or play_rosbag change
    if (!new_pointcloud_topic.empty() && new_pointcloud_topic != pointcloud_sub_->get_topic_name() && !safe_landing_planner_->play_rosbag_){
        RCLCPP_INFO(this->get_logger(), "Pointcloud topic changed to: %s", new_pointcloud_topic.c_str());
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            new_pointcloud_topic, rclcpp::SensorDataQoS(), std::bind(&SafeLandingPlannerNode::pointCloudCallback, this, std::placeholders::_1));
        if(raw_grid_sub_) raw_grid_sub_.reset(); // Remove raw grid sub if we switch to pointcloud
    }
    if (play_rosbag_changed && safe_landing_planner_->play_rosbag_){
        RCLCPP_INFO(this->get_logger(), "Switched to play_rosbag mode. Subscribing to /raw_grid_slp.");
        raw_grid_sub_ = this->create_subscription<safe_landing_planner::msg::SLPGridMsg>(
            "/raw_grid_slp", rclcpp::SystemDefaultsQoS(), std::bind(&SafeLandingPlannerNode::rawGridCallback, this, std::placeholders::_1));
        if(pointcloud_sub_) pointcloud_sub_.reset(); // Remove pointcloud sub
    } else if (play_rosbag_changed && !safe_landing_planner_->play_rosbag_ && !new_pointcloud_topic.empty()){
        RCLCPP_INFO(this->get_logger(), "Switched off play_rosbag mode. Subscribing to pointcloud topic: %s", new_pointcloud_topic.c_str());
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            new_pointcloud_topic, rclcpp::SensorDataQoS(), std::bind(&SafeLandingPlannerNode::pointCloudCallback, this, std::placeholders::_1));
        if(raw_grid_sub_) raw_grid_sub_.reset();
    }


    RCLCPP_INFO(this->get_logger(), "SafeLandingPlannerNode parameters reconfigured.");
    return result;
}


void SafeLandingPlannerNode::positionCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> guard(*cloud_msg_mutex_); // Mutex protects current_pose_ and previous_pose_
  previous_pose_ = current_pose_;
  current_pose_ = *msg;
  position_received_ = true;
}

void SafeLandingPlannerNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  {
    std::lock_guard<std::mutex> lck(*cloud_msg_mutex_);
    newest_cloud_msg_ = *msg;
  }
  if(cloud_ready_cv_) cloud_ready_cv_->notify_one();
}

void SafeLandingPlannerNode::rawGridCallback(const safe_landing_planner::msg::SLPGridMsg::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> transformed_cloud_guard(*transformed_cloud_mutex_);
  if(safe_landing_planner_) safe_landing_planner_->raw_grid_ = *msg;
  cloud_transformed_ = true; // Signal that data is ready (from rosbag)
}


void SafeLandingPlannerNode::cmdLoopCallback() {
  status_msg_.state = static_cast<uint8_t>(avoidance::MAV_STATE::MAV_STATE_ACTIVE);

  // Wait for cloud to be transformed if not playing from rosbag
  if (!safe_landing_planner_->play_rosbag_) {
    std::unique_lock<std::mutex> transformed_cloud_lock(*transformed_cloud_mutex_);
    if (!cloud_transformed_ && rclcpp::ok()) { // Check cloud_transformed_ before waiting
        if (cloud_ready_cv_->wait_for(transformed_cloud_lock, std::chrono::duration<double>(safe_landing_planner_->timeout_termination_)) == std::cv_status::timeout) {
            status_msg_.state = static_cast<uint8_t>(avoidance::MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Timeout waiting for transformed cloud in cmdLoop.");
            publishSystemStatus(); // Publish status immediately on timeout
            return; // Skip this cycle if cloud timed out
        }
    }
  } // else, if playing from rosbag, rawGridCallback sets cloud_transformed_

  rclcpp::Time now = this->now();
  rclcpp::Duration since_last_algo = now - last_algo_time_;
  rclcpp::Duration since_start = now - start_time_;
  checkFailsafe(since_last_algo, since_start);

  if(safe_landing_planner_ && current_pose_.header.stamp.sec > 0) { // Ensure current_pose is valid
    safe_landing_planner_->setPose(avoidance::toEigen(current_pose_.pose.position),
                                   avoidance::toEigen(current_pose_.pose.orientation));
    {
      std::lock_guard<std::mutex> transformed_cloud_guard(*transformed_cloud_mutex_);
      if (cloud_transformed_ || safe_landing_planner_->play_rosbag_) { // Ensure data is ready
        safe_landing_planner_->runSafeLandingPlanner();
        cloud_transformed_ = false; // Reset flag
      } else {
         RCLCPP_DEBUG(this->get_logger(), "Skipping planner run, no new transformed cloud and not in rosbag play mode.");
      }
    }
    visualizer_.visualizeSafeLandingPlanner(*(safe_landing_planner_.get()), current_pose_.pose.position,
                                            previous_pose_.pose.position,
                                            this->get_parameter("std_dev_threshold").as_double(),
                                            this->get_parameter("n_points_threshold").as_double() );
    publishSerialGrid();
    last_algo_time_ = this->now();
  } else {
     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Skipping planner iteration, current_pose not yet valid.");
  }


  if (this->now() - t_status_sent_ > std::chrono::duration<double>(0.2)) { // Use std::chrono for duration comparison
     publishSystemStatus();
  }
}

void SafeLandingPlannerNode::checkFailsafe(rclcpp::Duration since_last_algo, rclcpp::Duration since_start) {
  if (!safe_landing_planner_) return;
  rclcpp::Duration timeout_termination = rclcpp::Duration::from_seconds(safe_landing_planner_->timeout_termination_);
  rclcpp::Duration timeout_critical = rclcpp::Duration::from_seconds(safe_landing_planner_->timeout_critical_);

  if (since_last_algo > timeout_termination && since_start > timeout_termination) {
    status_msg_.state = static_cast<uint8_t>(avoidance::MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
  } else if (since_last_algo > timeout_critical && since_start > timeout_critical) {
    status_msg_.state = static_cast<uint8_t>(avoidance::MAV_STATE::MAV_STATE_CRITICAL);
  }
}

void SafeLandingPlannerNode::publishSystemStatus() {
  status_msg_.header.stamp = this->now();
  status_msg_.component = 196;
  if(mavros_system_status_pub_) mavros_system_status_pub_->publish(status_msg_);
  t_status_sent_ = this->now();
}

void SafeLandingPlannerNode::publishSerialGrid() {
  if (!grid_pub_ || !safe_landing_planner_) return;
  static int grid_seq_local = 0; // Keep local static counter if member grid_seq_ removed
  Grid prev_grid = safe_landing_planner_->getPreviousGrid(); // Assuming this is what should be published

  auto grid_msg = std::make_unique<safe_landing_planner::msg::SLPGridMsg>();
  grid_msg->header.frame_id = "local_origin"; // Should be a parameter
  grid_msg->header.stamp = this->now();
  grid_msg->seq = grid_seq_local++; // Use local static seq
  grid_msg->grid_size = prev_grid.getGridSize();
  grid_msg->cell_size = prev_grid.getCellSize();

  // Fill MultiArrayLayout
  auto fill_dim = [](std_msgs::msg::MultiArrayDimension& dim, const std::string& label, uint32_t size, uint32_t stride){
      dim.label = label;
      dim.size = size;
      dim.stride = stride;
  };

  uint32_t rows = prev_grid.mean_.rows();
  uint32_t cols = prev_grid.mean_.cols();

  grid_msg->mean.layout.dim.resize(2);
  fill_dim(grid_msg->mean.layout.dim[0], "rows", rows, rows * cols);
  fill_dim(grid_msg->mean.layout.dim[1], "cols", cols, cols);
  grid_msg->mean.layout.data_offset = 0;

  grid_msg->land.layout.dim.resize(2);
  fill_dim(grid_msg->land.layout.dim[0], "rows", rows, rows * cols);
  fill_dim(grid_msg->land.layout.dim[1], "cols", cols, cols);
  grid_msg->land.layout.data_offset = 0;

  Eigen::MatrixXf variance = prev_grid.getVariance();
  grid_msg->std_dev.layout.dim.resize(2);
  fill_dim(grid_msg->std_dev.layout.dim[0], "rows", rows, rows * cols);
  fill_dim(grid_msg->std_dev.layout.dim[1], "cols", cols, cols);
  grid_msg->std_dev.layout.data_offset = 0;

  Eigen::MatrixXi counter = prev_grid.getCounter();
  grid_msg->counter.layout.dim.resize(2);
  fill_dim(grid_msg->counter.layout.dim[0], "rows", rows, rows * cols);
  fill_dim(grid_msg->counter.layout.dim[1], "cols", cols, cols);
  grid_msg->counter.layout.data_offset = 0;

  grid_msg->mean.data.reserve(rows * cols);
  grid_msg->land.data.reserve(rows * cols);
  grid_msg->std_dev.data.reserve(rows * cols);
  grid_msg->counter.data.reserve(rows * cols);

  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      grid_msg->mean.data.push_back(prev_grid.mean_(i, j));
      grid_msg->land.data.push_back(prev_grid.land_(i, j));
      grid_msg->std_dev.data.push_back(sqrtf(variance(i, j)));
      grid_msg->counter.data.push_back(counter(i, j));
    }
  }
  Eigen::Vector2i pos_index = safe_landing_planner_->getPositionIndex();
  grid_msg->curr_pos_index.x = static_cast<float>(pos_index.x());
  grid_msg->curr_pos_index.y = static_cast<float>(pos_index.y());

  grid_pub_->publish(std::move(grid_msg));
}

void SafeLandingPlannerNode::pointCloudTransformThread() {
  while (rclcpp::ok() && !should_exit_) {
    std::unique_lock<std::mutex> cloud_msg_lock(*(cloud_msg_mutex_));
    if (cloud_ready_cv_->wait_for(cloud_msg_lock, std::chrono::milliseconds(500),
                                 [&]{ return newest_cloud_msg_.header.stamp.sec != 0 || should_exit_; })) { // Wait if stamp is zero or exit
      if (should_exit_) break;

      sensor_msgs::msg::PointCloud2 cloud_to_transform = newest_cloud_msg_; // Copy
      newest_cloud_msg_.header.stamp.sec = 0; newest_cloud_msg_.header.stamp.nanosec = 0; // Mark as consumed
      cloud_msg_lock.unlock(); // Unlock while transforming

      if (tf_buffer_ && tf_buffer_->_frameExists("local_origin") && tf_buffer_->_frameExists(cloud_to_transform.header.frame_id)) {
        try {
          geometry_msgs::msg::TransformStamped transform_stamped;
          transform_stamped = tf_buffer_->lookupTransform("local_origin", cloud_to_transform.header.frame_id,
                               tf2_ros::fromMsg(cloud_to_transform.header.stamp), rclcpp::Duration::from_seconds(0.1)); // Use fromMsg to convert time

          pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
          pcl::fromROSMsg(cloud_to_transform, pcl_cloud);

          std::vector<int> dummy_index; // removeNaNFromPointCloud needs an index vector
          pcl::removeNaNFromPointCloud(pcl_cloud, pcl_cloud, dummy_index);

          Eigen::Isometry3d eigen_transform = tf2::transformToEigen(transform_stamped);
          pcl::PointCloud<pcl::PointXYZ> transformed_pcl_cloud;
          pcl::transformPointCloud(pcl_cloud, transformed_pcl_cloud, eigen_transform.cast<float>());

          std::lock_guard<std::mutex> transformed_cloud_guard(*(transformed_cloud_mutex_));
          cloud_transformed_ = true;
          if(safe_landing_planner_) safe_landing_planner_->cloud_ = std::move(transformed_pcl_cloud); // Pass PCL cloud

        } catch (const tf2::TransformException &ex) {
          RCLCPP_ERROR(this->get_logger(), "TF Exception in pointCloudTransformThread: %s", ex.what());
        }
      } else {
         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF frames not available for point cloud transform.");
      }
       if(cloud_ready_cv_) cloud_ready_cv_->notify_one(); // Notify main loop that processing (or attempt) is done
    } else { // Timeout or spurious wakeup
        if (should_exit_) break;
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Timeout waiting for new cloud message in transform thread.");
    }
  }
}
} // namespace avoidance

// Main function is in safe_landing_planner_node_main.cpp
// No RCLCPP_COMPONENTS_REGISTER_NODE here if it's not meant to be a component loaded by class name.
// If it can be run standalone OR as a component, the main.cpp handles standalone,
// and this would need the macro if it were to be registered.
// For now, assuming it's primarily run via its own main.
