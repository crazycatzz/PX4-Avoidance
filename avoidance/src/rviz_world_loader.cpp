#include "avoidance/rviz_world_loader.h"

// #include <ros/console.h> // Replaced by rclcpp/logging.hpp (implicitly via rclcpp.hpp)

namespace avoidance {

// Constructor updated to take rclcpp::NodeOptions
WorldVisualizer::WorldVisualizer(const rclcpp::NodeOptions & options, const std::string& nodelet_ns)
    : Node("rviz_world_loader_node", options), nodelet_ns_(nodelet_ns) { // Call base Node constructor

  // Subscription
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "mavros/local_position/pose", 1, // QoS profile can be more specific, e.g. rclcpp::SensorDataQoS()
    std::bind(&WorldVisualizer::positionCallback, this, std::placeholders::_1));

  // Publishers
  world_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("world", 1); // QoS profile can be more specific
  drone_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("drone", 1);    // QoS profile can be more specific

  // Timer
  loop_timer_ = this->create_wall_timer(
    std::chrono::seconds(2), // ROS2 uses std::chrono for durations
    std::bind(&WorldVisualizer::loopCallback, this));

  // Parameters
  std::string world_name_param = "world_name";
  if (!nodelet_ns_.empty()) {
     // Nodelet ns is usually handled by node namespace or remapping in ROS2.
     // For parameters, it's simpler to use a non-namespaced parameter name
     // or retrieve it with the node's namespace if that's the intent.
     // If nodelet_ns_ is truly a sub-namespace for parameters:
     // world_name_param = nodelet_ns_ + ".world_name"; // ROS2 uses '.' for sub-parameters
  }
  this->declare_parameter<std::string>(world_name_param, "");
  this->get_parameter(world_name_param, world_path_);
  RCLCPP_INFO(this->get_logger(), "World path: %s", world_path_.c_str());
}

// loopCallback signature changed in header (no TimerEvent)
void WorldVisualizer::loopCallback() {
  // visualize world in RVIZ
  if (!world_path_.empty()) {
    if (visualizeRVIZWorld(world_path_)) {
      RCLCPP_WARN(this->get_logger(), "[WorldVisualizer] Failed to visualize Rviz world");
    }
  }
}

int WorldVisualizer::resolveUri(std::string& uri) {
  // Iterate through all locations in GAZEBO_MODEL_PATH
  // This function is largely system-dependent and might not need ROS2 changes directly
  // unless environment variable names or typical paths change.
  char* gazebo_model_path_cstr = getenv("GAZEBO_MODEL_PATH");
  std::string gazebo_model_path = gazebo_model_path_cstr ? gazebo_model_path_cstr : ""; // Handle null
  char* home_cstr = getenv("HOME");
  std::string home_path = home_cstr ? home_cstr : ""; // Handle null

  if (uri.rfind("model://", 0) == 0) { // Check if string starts with model://
    uri = uri.substr(8); // Length of "model://"
  } else {
    // If URI is not model://, perhaps it's already a file path or other scheme
    // This part of logic might need review based on how URIs are actually formed
    RCLCPP_DEBUG(this->get_logger(), "URI does not start with model:// %s", uri.c_str());
  }

  std::stringstream all_locations;
  if (!gazebo_model_path.empty()) {
    all_locations << gazebo_model_path;
  }
  if (!home_path.empty()) {
    if (!gazebo_model_path.empty()) all_locations << ":";
    all_locations << home_path << "/.gazebo/models"; // Typical Gazebo path
    all_locations << ":" << home_path << "/.ignition/fuel"; // Typical Ignition path
  }

  std::string current_location;
  while (getline(all_locations, current_location, ':')) {
    struct stat s;
    std::string temp = current_location + uri;
    if (stat(temp.c_str(), &s) == 0) {
      if (s.st_mode & S_IFREG)  // this path describes a file
      {
        uri = "file://" + current_location + uri;
        return 0;
      }
    }
  }
  return 1;
}

int WorldVisualizer::visualizeRVIZWorld(const std::string& world_path) {
  std::ifstream fin(world_path);
  YAML::Node doc = YAML::Load(fin);
  size_t object_counter = 0;
  visualization_msgs::MarkerArray marker_array;

  for (YAML::const_iterator it = doc.begin(); it != doc.end(); ++it) {
    const YAML::Node& node = *it;
    world_object item;
    node >> item;
    object_counter++;

    // convert object to marker
    visualization_msgs::msg::Marker m; // Use msg namespace
    m.header.frame_id = item.frame_id;
    m.header.stamp = this->now(); // Use node's clock

    if (item.type == "mesh") {
      // model URI resolving logic might need adjustment if GAZEBO_MODEL_PATH structure changes
      // or if using Fuel servers more directly in ROS2/Ignition.
      if (item.mesh_resource.rfind("model://", 0) == 0) {
        if (resolveUri(item.mesh_resource)) {
          RCLCPP_ERROR(this->get_logger(), "RVIZ world loader could not find model: %s", item.mesh_resource.c_str());
          return 1;
        }
      }
      m.type = visualization_msgs::msg::Marker::MESH_RESOURCE; // Use msg namespace
      m.mesh_resource = item.mesh_resource;
      m.mesh_use_embedded_materials = true;
    } else if (item.type == "cube") {
      m.type = visualization_msgs::msg::Marker::CUBE; // Use msg namespace
      m.color.a = 0.9f; // Use f suffix for float
      m.color.r = 0.5f;
      m.color.g = 0.5f;
      m.color.b = 0.5f;
    } else if (item.type == "sphere") {
      m.type = visualization_msgs::msg::Marker::SPHERE; // Use msg namespace
      m.color.a = 0.9f;
      m.color.r = 0.5f;
      m.color.g = 0.5f;
      m.color.b = 0.5f;
    } else if (item.type == "cylinder") {
      m.type = visualization_msgs::msg::Marker::CYLINDER; // Use msg namespace
      m.color.a = 0.9f;
      m.color.r = 0.5f;
      m.color.g = 0.5f;
      m.color.b = 0.5f;
    } else {
      RCLCPP_ERROR(this->get_logger(), "RVIZ world loader invalid object type in yaml file: %s", item.type.c_str());
      return 1;
    }

    m.scale.x = item.scale.x();
    m.scale.y = item.scale.y();
    m.scale.z = item.scale.z();
    m.pose.position.x = item.position.x();
    m.pose.position.y = item.position.y();
    m.pose.position.z = item.position.z();
    m.pose.orientation.x = item.orientation.x();
    m.pose.orientation.y = item.orientation.y();
    m.pose.orientation.z = item.orientation.z();
    m.pose.orientation.w = item.orientation.w();
    m.id = static_cast<int32_t>(object_counter); // Marker ID is int32
    m.lifetime = rclcpp::Duration(0, 0); // Zero duration means infinite
    m.action = visualization_msgs::msg::Marker::ADD; // Use msg namespace
    marker_array.markers.push_back(m);
  }

  if (object_counter != marker_array.markers.size()) {
    RCLCPP_ERROR(this->get_logger(), "Could not display all world objects");
  }

  world_pub_->publish(marker_array);
  RCLCPP_INFO_ONCE(this->get_logger(), "Successfully loaded rviz world");
  return 0;
}

// positionCallback signature changed in header to use ConstSharedPtr
int WorldVisualizer::visualizeDrone(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose) {
  visualization_msgs::msg::Marker drone; // Use msg namespace
  drone.header.frame_id = "local_origin"; // Ensure this frame_id is appropriate for your TF tree
  drone.header.stamp = this->now(); // Use node's clock
  drone.type = visualization_msgs::msg::Marker::MESH_RESOURCE; // Use msg namespace
  drone.mesh_resource = "model://matrice_100/meshes/Matrice_100.dae";
  if (drone.mesh_resource.rfind("model://", 0) == 0) {
    if (resolveUri(drone.mesh_resource)) {
      RCLCPP_ERROR(this->get_logger(), "RVIZ world loader could not find drone model: %s", drone.mesh_resource.c_str());
      return 1;
    }
  }
  drone.mesh_use_embedded_materials = true;
  drone.scale.x = 1.5;
  drone.scale.y = 1.5f;
  drone.scale.z = 1.5f;
  drone.pose.position.x = pose->pose.position.x; // Access via pointer
  drone.pose.position.y = pose->pose.position.y;
  drone.pose.position.z = pose->pose.position.z;
  drone.pose.orientation.x = pose->pose.orientation.x;
  drone.pose.orientation.y = pose->pose.orientation.y;
  drone.pose.orientation.z = pose->pose.orientation.z;
  drone.pose.orientation.w = pose->pose.orientation.w;
  drone.id = 0;
  drone.lifetime = rclcpp::Duration(0, 0); // Zero duration means infinite
  drone.action = visualization_msgs::msg::Marker::ADD; // Use msg namespace

  drone_pub_->publish(drone);

  return 0;
}

// positionCallback signature changed in header to use ConstSharedPtr
void WorldVisualizer::positionCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  // visualize drone in RVIZ
  if (!world_path_.empty()) {
    if (visualizeDrone(msg)) { // Pass the ConstSharedPtr
      RCLCPP_WARN(this->get_logger(), "Failed to visualize drone in RViz");
    }
  }
}

// extraction operators
// These should be fine as they operate on YAML::Node and Eigen types
void operator>>(const YAML::Node& node, Eigen::Vector3f& v) {
  v.x() = node[0].as<float>();
  v.y() = node[1].as<float>();
  v.z() = node[2].as<float>();
}

void operator>>(const YAML::Node& node, Eigen::Vector4f& v) {
  v.x() = node[0].as<float>();
  v.y() = node[1].as<float>();
  v.z() = node[2].as<float>();
  v.w() = node[3].as<float>();
}

void operator>>(const YAML::Node& node, world_object& item) {
  item.type = node["type"].as<std::string>();
  item.name = node["name"].as<std::string>();
  item.frame_id = node["frame_id"].as<std::string>();
  item.mesh_resource = node["mesh_resource"].as<std::string>();
  node["position"] >> item.position;
  node["orientation"] >> item.orientation;
  node["scale"] >> item.scale;
}
}
