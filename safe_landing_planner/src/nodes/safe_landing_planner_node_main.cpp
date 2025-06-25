#include "safe_landing_planner/safe_landing_planner_node.hpp" // Includes rclcpp.hpp
#include <memory> // For std::make_shared

int main(int argc, char **argv) {
  // using namespace avoidance; // Namespace is used for the class, not needed for rclcpp functions
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto safe_landing_planner_node = std::make_shared<avoidance::SafeLandingPlannerNode>(options);

  // The worker thread is started in the constructor of SafeLandingPlannerNode
  // and should be joined in its destructor.
  // rclcpp::spin will block until the node is shut down.
  rclcpp::spin(safe_landing_planner_node);

  rclcpp::shutdown();
  return 0;
}
