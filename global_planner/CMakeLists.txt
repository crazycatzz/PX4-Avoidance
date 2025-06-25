cmake_minimum_required(VERSION 3.16)
project(global_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) # From package.xml depend
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap_msgs REQUIRED) # From package.xml depend
find_package(octomap_ros REQUIRED)  # From package.xml depend
find_package(OctoMap REQUIRED)     # For OctoMap libraries
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(avoidance REQUIRED)    # Workspace dependency
find_package(rosidl_default_generators REQUIRED) # For message generation

find_package(yaml-cpp REQUIRED) # Added as it was in original include_directories

ament_cmake_auto_set_compiler_flags()

# Removed DISABLE_SIMULATION block, assuming not applicable or needs rework for ROS2

################################################
## Declare ROS messages, services and actions ##
################################################
# Assumes PathWithRiskMsg.msg is moved to a 'msg' subdirectory
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PathWithRiskMsg.msg"
  DEPENDENCIES std_msgs geometry_msgs # Dependencies from original generate_messages
)
# Create a variable that other targets can depend on for message generation
set(${PROJECT_NAME}_MSG_TARGETS rosidl_typesupport_cpp::${PROJECT_NAME})


# Dynamic reconfigure (generate_dynamic_reconfigure_options) is removed.
# Parameters should be handled using ROS2 parameter system (YAML files, declare_parameter in code).

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

## Declare a C++ library
add_library(global_planner SHARED
  src/library/node.cpp
  src/library/cell.cpp
  src/library/global_planner.cpp
  src/nodes/global_planner_node.cpp
)

target_include_directories(global_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OctoMap_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

ament_target_dependencies(global_planner PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  octomap_msgs # For OctoMap messages
  octomap_ros  # For OctoMap ROS integration
  mavros
  mavros_msgs
  avoidance # Assuming avoidance exports 'avoidance::avoidance' or similar
  ${${PROJECT_NAME}_MSG_TARGETS} # Dependency for generated messages
)

target_link_libraries(global_planner PUBLIC
  ${PCL_LIBRARIES}
  ${OctoMap_LIBRARIES}
  ${yaml-cpp_LIBRARIES}
)

## Declare a C++ executable
add_executable(global_planner_node src/nodes/global_planner_node_main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(global_planner_node
  global_planner
)
# Ensure the executable also has access to ROS dependencies if not using global_planner as PRIVATE/PUBLIC link
ament_target_dependencies(global_planner_node PUBLIC rclcpp)


#############
## Install ##
#############
install(TARGETS global_planner global_planner_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # Install executables to lib/project_name
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install generated message headers and CMake files (handled by rosidl_generate_interfaces and ament_package)

# Export include directories, libraries and dependencies
ament_export_include_directories(include)
ament_export_libraries(global_planner) # Export the library
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  octomap_msgs
  octomap_ros
  mavros
  mavros_msgs
  avoidance
  rosidl_default_runtime # For messages
)

install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# Install parameter files if any (e.g. from cfg/GlobalPlannerNode.cfg conversion)
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/config
#   FILES_MATCHING PATTERN "*.yaml"
# )

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}-test test/main.cpp
                                      test/test_example.cpp) # Assuming these are the test files

  target_link_libraries(${PROJECT_NAME}-test PRIVATE
    global_planner # Link against the library being tested
    ${PCL_LIBRARIES}
    ${OctoMap_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
  )
  # Ensure ROS dependencies are available to the test target
  ament_target_dependencies(${PROJECT_NAME}-test PUBLIC
    rclcpp # Or other specific ROS deps needed by tests
    # Add other dependencies like std_msgs, geometry_msgs if test code uses them directly
  )

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
          COMMAND ${CMAKE_CTEST_COMMAND} --force-new-ctest-process --output-on-failure # Run tests via ctest
          COMMAND lcov --capture --no-external --directory ${PROJECT_BINARY_DIR} --base-directory ${CMAKE_CURRENT_SOURCE_DIR} --output-file test_coverage.info --rc lcov_branch_coverage=1
          COMMAND lcov -a base_coverage.info -a test_coverage.info -o coverage.info --rc lcov_branch_coverage=1
          COMMAND lcov --rc lcov_branch_coverage=1 --summary coverage.info
          WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
          DEPENDS ${PROJECT_NAME}-test
      )
      add_custom_target(${PROJECT_NAME}-test_coverage_html
          COMMAND genhtml coverage.info --output-directory out --branch-coverage
          WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
          DEPENDS ${PROJECT_NAME}-test_coverage
      )
  endif()
endif()
