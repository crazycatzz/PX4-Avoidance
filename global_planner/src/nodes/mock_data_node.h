#ifndef GLOBAL_PLANNER_MOCK_DATA_NODE_H
#define GLOBAL_PLANNER_MOCK_DATA_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp> // Often same path in ROS2
#include <geometry_msgs/msg/pose_stamped.hpp> // For assumed publisher types

#include <stdlib.h> // For rand, srand
#include <vector>

#include "global_planner/common.h"

namespace global_planner {

class MockDataNode : public rclcpp::Node { // Inherit from rclcpp::Node
 public:
  explicit MockDataNode(const rclcpp::NodeOptions & options); // Added constructor
  ~MockDataNode();
  void createWall(int dist, int width, int height);
  void sendClickedPoint();
  void receivePath(const nav_msgs::msg::Path::ConstSharedPtr msg); // Updated signature
  void sendMockData();

  std::vector<float> points_{5.5f, -0.5f, 0.5f, 5.5f, 0.5f, 0.5f,  5.5f, 1.5f, 0.5f, 5.5f, -0.5f, 1.5f, 5.5f, 0.5f,
                             1.5f, 5.5f,  1.5f, 1.5f, 5.5f, -0.5f, 2.5f, 5.5f, 0.5f, 2.5f, 5.5f,  1.5f, 2.5f}; // Added 'f' suffix

 private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_points_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr global_goal_pub_;

  rclcpp::TimerBase::SharedPtr mock_data_timer_;
  rclcpp::TimerBase::SharedPtr clicked_point_timer_;
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_MOCK_DATA_NODE_H
