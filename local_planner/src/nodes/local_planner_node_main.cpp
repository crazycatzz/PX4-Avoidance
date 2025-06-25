#include "local_planner/local_planner_nodelet.h" // Includes rclcpp.hpp and component registration
#include <rclcpp/rclcpp.hpp>
#include <memory> // For std::make_shared

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  // Configure node options if necessary, e.g., for parameters:
  options.automatically_declare_parameters_from_overrides(true);

  auto local_planner_nodelet = std::make_shared<avoidance::LocalPlannerNodelet>(options);

  rclcpp::spin(local_planner_nodelet->get_node_base_interface()); // Spin the component

  rclcpp::shutdown();
  return 0;
}