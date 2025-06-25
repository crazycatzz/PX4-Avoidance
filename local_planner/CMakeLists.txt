cmake_minimum_required(VERSION 3.16)
project(local_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED) # For PCL type conversions
find_package(PCL REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(avoidance REQUIRED)
find_package(rclcpp_components REQUIRED) # For composable nodes
find_package(Boost REQUIRED COMPONENTS system) # Add other components if needed e.g. thread
find_package(yaml-cpp REQUIRED) # For parameters or other YAML usage

ament_cmake_auto_set_compiler_flags()

# Removed DISABLE_SIMULATION block, assuming not applicable or needs rework for ROS2

# No messages are generated in this package based on original CMakeLists.txt

# Dynamic reconfigure (generate_dynamic_reconfigure_options) is removed.
# Parameters should be handled using ROS2 parameter system (YAML files, declare_parameter in code).
# The LocalPlannerNode.cfg would need to be converted to a YAML file.

###################################
## ament specific configuration ##
###################################
ament_package()

###########
## Build ##
###########

## CMake Setup
# Build in Release mode if nothing is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

## Library: local_planner (core logic)
set(LOCAL_PLANNER_CPP_FILES
  "src/nodes/local_planner.cpp"
  "src/nodes/waypoint_generator.cpp"
  "src/nodes/tree_node.cpp"
  "src/nodes/star_planner.cpp"
  "src/nodes/planner_functions.cpp"
  "src/nodes/local_planner_visualization.cpp"
  "src/utils/trajectory_simulator.cpp"
)
add_library(local_planner SHARED ${LOCAL_PLANNER_CPP_FILES})

target_include_directories(local_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(local_planner PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  mavros
  mavros_msgs
  avoidance # Assuming avoidance exports avoidance::avoidance or similar
  # yaml-cpp is linked below
)
target_link_libraries(local_planner PUBLIC
  ${PCL_LIBRARIES}
  ${yaml-cpp_LIBRARIES} # Provided by find_package(yaml-cpp)
  ${Boost_LIBRARIES}    # Provided by find_package(Boost)
)

## Library: local_planner_nodelet (specific nodelet/component implementation)
# This library contains the class that might have been a nodelet.
# In ROS2, if it's a composable node, it should derive from rclcpp::Node.
add_library(local_planner_component SHARED src/nodes/local_planner_nodelet.cpp) # Renamed for clarity

target_include_directories(local_planner_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # If it has its own headers or uses project's
  $<INSTALL_INTERFACE:include>
  # Include dirs from other dependencies are typically handled by ament_target_dependencies
)
ament_target_dependencies(local_planner_component PUBLIC
  rclcpp # Add other specific ROS deps if needed directly by this component
  local_planner # Depends on the core logic library
)
# Linking for local_planner_component will also pull transitive dependencies from local_planner

# If local_planner_nodelet.cpp contains RCLCPP_COMPONENTS_REGISTER_NODE,
# then this library can be loaded by a component container.
# We need to make sure it's installed correctly for that.
# rclcpp_components_register_nodes(local_planner_component PLUGIN "local_planner::LocalPlannerNodelet")


## Executable: local_planner_node (main entry point, possibly loads the component or uses its class)
add_executable(local_planner_node src/nodes/local_planner_node_main.cpp)

ament_target_dependencies(local_planner_node PUBLIC
  rclcpp # For main function
  local_planner_component # Links against the component library
)
# Transitive dependencies from local_planner_component (including local_planner) will be linked.

#############
## Install ##
#############
install(TARGETS local_planner local_planner_component local_planner_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # Install executables
)

install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h"
)

# If local_planner_component is a true composable node plugin, register and install it.
# This requires local_planner_nodelet.cpp to use RCLCPP_COMPONENTS_REGISTER_NODE.
# If it is, uncomment the following:
# rclcpp_components_register_nodes(local_planner_component PLUGIN "local_planner::LocalPlannerNodelet")
# install(TARGETS local_planner_component
#   ARCHIVE DESTINATION lib/local_planner # Or another component specific dir
#   LIBRARY DESTINATION lib/local_planner
# )


ament_export_include_directories(include)
ament_export_libraries(local_planner local_planner_component)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  mavros
  mavros_msgs
  avoidance
  rclcpp_components # If exporting components
  # yaml-cpp is not typically exported as a ROS dep, Boost neither
)

install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# Install parameter files (converted from .cfg)
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/config
#   FILES_MATCHING PATTERN "*.yaml"
# )

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Finds gtest, linters

  set(LOCAL_PLANNER_MAIN_TEST_FILES test/main.cpp
                                    test/test_example.cpp # Keep if relevant
                                    test/test_local_planner.cpp
                                    test/test_planner_functions.cpp
                                    test/test_star_planner.cpp
                                    test/test_trajectory_simulator.cpp
                                    test/test_waypoint_generator.cpp
  )
  ament_add_gtest(${PROJECT_NAME}-test "${LOCAL_PLANNER_MAIN_TEST_FILES}")
  target_link_libraries(${PROJECT_NAME}-test PRIVATE
    local_planner
    ${PCL_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  ament_target_dependencies(${PROJECT_NAME}-test PUBLIC rclcpp) # For ROS utilities in tests

  set(LOCAL_PLANNER_NODELET_TEST_FILES test/main.cpp # Re-evaluate if main is needed for both
                                         test/test_local_planner_nodelet.cpp
  )
  ament_add_gtest(${PROJECT_NAME}-component-test "${LOCAL_PLANNER_NODELET_TEST_FILES}") # Renamed target
  target_link_libraries(${PROJECT_NAME}-component-test PRIVATE
    local_planner_component
    local_planner # Often the component test will need the base lib too
    ${PCL_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  ament_target_dependencies(${PROJECT_NAME}-component-test PUBLIC rclcpp)


  if (${CMAKE_BUILD_TYPE} STREQUAL "Coverage")
      SET(CMAKE_CXX_FLAGS_INIT "${CMAKE_CXX_FLAGS}" CACHE STRING "" FORCE)
      SET(CMAKE_C_FLAGS_INIT "${CMAKE_C_FLAGS}" CACHE STRING "" FORCE)
      SET(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS}" CACHE STRING "" FORCE)

      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_INIT} -g -O0 -fprofile-arcs -ftest-coverage --coverage")
      SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS_INIT} -g -O0 -fprofile-arcs -ftest-coverage --coverage")
      SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS_INIT} --coverage")

      add_custom_target(${PROJECT_NAME}-test_coverage
          COMMAND lcov --zerocounters --directory ${PROJECT_BINARY_DIR}
          COMMAND lcov --capture --initial --no-external --directory ${PROJECT_BINARY_DIR} --base-directory ${CMAKE_CURRENT_SOURCE_DIR} --output-file base_coverage.info --rc lcov_branch_coverage=1
          COMMAND ${CMAKE_CTEST_COMMAND} --force-new-ctest-process --output-on-failure # Run all tests
          COMMAND lcov --capture --no-external --directory ${PROJECT_BINARY_DIR} --base-directory ${CMAKE_CURRENT_SOURCE_DIR} --output-file test_coverage.info --rc lcov_branch_coverage=1
          COMMAND lcov -a base_coverage.info -a test_coverage.info -o coverage.info --rc lcov_branch_coverage=1
          COMMAND lcov --rc lcov_branch_coverage=1 --summary coverage.info
          WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
          DEPENDS ${PROJECT_NAME}-test ${PROJECT_NAME}-component-test
      )
      add_custom_target(${PROJECT_NAME}-test_coverage_html
          COMMAND genhtml coverage.info --output-directory out --branch-coverage
          WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
          DEPENDS ${PROJECT_NAME}-test_coverage
      )
  endif()
endif()
