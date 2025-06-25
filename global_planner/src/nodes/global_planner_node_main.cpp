#include "global_planner/global_planner_node.h" // This header now includes rclcpp.hpp
#include <memory> // For std::make_shared

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Node options can be configured here if needed, e.g., for automatic parameter loading from a YAML file.
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true); // Good practice

  auto global_planner_node = std::make_shared<global_planner::GlobalPlannerNode>(options);

  rclcpp::spin(global_planner_node);
  rclcpp::shutdown();
  return 0;
}
