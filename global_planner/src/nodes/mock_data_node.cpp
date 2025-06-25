#include "mock_data_node.h"
#include <chrono> // For std::chrono literals

// Note: geometry_msgs/msg/point_stamped.hpp is now included via mock_data_node.h

namespace global_planner {

MockDataNode::MockDataNode(const rclcpp::NodeOptions & options) : Node("mock_data_node", options) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "global_path", 10, // Using QoS 10 as a default
      std::bind(&MockDataNode::receivePath, this, std::placeholders::_1));

  depth_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera/depth/points", 10);
  local_position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", 10);
  global_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("clicked_point", 10);

  createWall(5, 5, 6);

  // Replace loop with timers
  // Timer for sendMockData (e.g., every 1 second)
  mock_data_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MockDataNode::sendMockData, this));

  // One-shot timer for sendClickedPoint after 10 seconds
  // (Original logic was num_loops++ == 10 with 1Hz rate)
  clicked_point_timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      [this]() -> void { // Lambda to call it once and then cancel timer
        if (this->clicked_point_timer_) { // Check if timer is still valid
            this->sendClickedPoint();
            this->clicked_point_timer_->cancel(); // Make it one-shot
        }
      });
}

MockDataNode::~MockDataNode() {}

void MockDataNode::createWall(int dist, int width, int height) {
  points_.clear();
  for (int i = -width; i <= width; ++i) {
    for (int j = 0; j <= height; ++j) {
      points_.push_back(dist + 0.5);
      points_.push_back(i + 0.5);
      points_.push_back(j + 0.5);
    }
  }
}

void MockDataNode::sendClickedPoint() {
  geometry_msgs::msg::PointStamped msg; // Updated type
  msg.header.stamp = this->now(); // Set timestamp
  msg.header.frame_id = "world";
  msg.point.x = 8.5;
  msg.point.y = 4.5;
  msg.point.z = 1.5;
  global_goal_pub_->publish(msg);
}

void MockDataNode::receivePath(const nav_msgs::msg::Path::ConstSharedPtr msg) { // Updated signature
  for (const auto& p : msg->poses) { // Use msg->poses
    double x = p.pose.position.x;
    double y = p.pose.position.y;
    double z = p.pose.position.z;
    printf("(%2.2f, %2.2f, %2.2f) -> ", x, y, z);
  }
  printf("\n\n");
}

void MockDataNode::sendMockData() {
  // Create a PointCloud2
  sensor_msgs::msg::PointCloud2 cloud_msg; // Updated type
  cloud_msg.header.stamp = this->now(); // Set timestamp
  cloud_msg.header.frame_id = "world";
  // Fill some internals of the PointCloud2 like the header/width/height ...
  cloud_msg.height = 1;
  // cloud_msg.width = 4; // Width will be set by modifier.resize() or implicitly by number of points

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  // Set fields for x, y, z, and rgb. RGB is often packed into a single float.
  // Or handle as separate r, g, b uint8 fields.
  // The original used "rgb" as FLOAT32, which is common for packed color.
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                   "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                   "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                   "rgb", 1, sensor_msgs::msg::PointField::UINT32); // Changed rgb to UINT32 for packed color

  int n = points_.size() / 3;
  modifier.resize(n); // Resize to actual number of points

  // Define some raw data we'll put in the PointCloud2
  // uint8_t color_data[] = {40, 200, 120}; // Example color

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb(cloud_msg, "rgb"); // Iterator for packed RGB

  // Pack RGB into a single uint32_t: (R << 16) | (G << 8) | B
  uint8_t r = 40, g = 200, b = 120;
  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

  for (size_t i = 0; i < n; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
    *iter_x = points_[3 * i + 0];
    *iter_y = points_[3 * i + 1];
    *iter_z = points_[3 * i + 2];
    *iter_rgb = rgb;
  }

  depth_points_pub_->publish(cloud_msg);

  // Send position
  geometry_msgs::msg::PoseStamped pos; // Updated type
  pos.header.stamp = this->now(); // Set timestamp
  pos.header.frame_id = "world";
  pos.pose.position.x = 0.5;
  pos.pose.position.y = 2.5;
  pos.pose.position.z = 1.5;
  pos.pose.orientation.w = 1.0;
  local_position_pub_->publish(pos);
}

// Private member for timer, if it needs to be cancelled (like the one-shot)
// rclcpp::TimerBase::SharedPtr mock_data_timer_; // Declare in header if needed for sendMockData
// rclcpp::TimerBase::SharedPtr clicked_point_timer_; // Declare in header

}  // namespace global_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options; // Can configure options here if needed
  auto mock_data_node = std::make_shared<global_planner::MockDataNode>(options);
  rclcpp::spin(mock_data_node);
  rclcpp::shutdown();
  return 0;
}
