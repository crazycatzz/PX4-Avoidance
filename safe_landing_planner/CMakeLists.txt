cmake_minimum_required(VERSION 3.16)
project(safe_landing_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(avoidance REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED) # For parameter files

ament_cmake_auto_set_compiler_flags()

# Removed DISABLE_SIMULATION block

## Generate messages in the 'msg' folder
# Assumes SLPGridMsg.msg is moved to a 'msg' subdirectory
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SLPGridMsg.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
# Create a variable that other targets can depend on for message generation
set(${PROJECT_NAME}_MSG_TARGETS rosidl_typesupport_cpp::${PROJECT_NAME})


# Dynamic reconfigure (generate_dynamic_reconfigure_options) is removed for:
# cfg/SafeLandingPlannerNode.cfg
# cfg/WaypointGeneratorNode.cfg
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

## Library: safe_landing_planner (core logic)
# Note: Original SAFE_LANDING_PLANNER_CPP_FILES included waypoint_generator_node.cpp.
# This is unusual for a library if waypoint_generator_node.cpp contains a main().
# Assuming waypoint_generator.cpp contains the logic, and waypoint_generator_node.cpp is a main file.
set(SAFE_LANDING_PLANNER_LIB_SRC
  "src/nodes/safe_landing_planner.cpp" # Core SLP logic
  "src/nodes/waypoint_generator.cpp"   # Core Waypoint Generator logic
  "src/nodes/safe_landing_planner_visualization.cpp"
  # "src/nodes/safe_landing_planner_node.cpp" # This should be a main or part of executable
  # "src/nodes/waypoint_generator_node.cpp" # This should be a main or part of executable
)
add_library(safe_landing_planner SHARED ${SAFE_LANDING_PLANNER_LIB_SRC})

target_include_directories(safe_landing_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS} # If yaml-cpp is used directly by library code
)
ament_target_dependencies(safe_landing_planner PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  mav_msgs
  tf2_ros
  pcl_conversions
  mavros
  mavros_msgs
  avoidance # Assuming avoidance exports target e.g. avoidance::avoidance
  ${${PROJECT_NAME}_MSG_TARGETS} # For SLPGridMsg
)
target_link_libraries(safe_landing_planner PUBLIC
  ${PCL_LIBRARIES}
  ${yaml-cpp_LIBRARIES} # If yaml-cpp is used directly by library code
)

## Executable: safe_landing_planner_node
# Assuming safe_landing_planner_node.cpp (from original lib files) contains the node logic
# and safe_landing_planner_node_main.cpp is the main.
add_executable(safe_landing_planner_node
  src/nodes/safe_landing_planner_node_main.cpp
  src/nodes/safe_landing_planner_node.cpp # Added here, assuming it's not just main
)
ament_target_dependencies(safe_landing_planner_node PUBLIC
  rclcpp
  safe_landing_planner # Links against the main library
)
# target_link_libraries(safe_landing_planner_node PUBLIC safe_landing_planner) # Alternative way

## Executable: waypoint_generator_node
# Assuming waypoint_generator_node.cpp (from original lib files) contains the node logic
# and is also the main, or there's another main file for it.
# The original CMakeLists adds waypoint_generator_node.cpp as an executable source.
add_executable(waypoint_generator_node
  src/nodes/waypoint_generator_node.cpp # This was in LIB before, now assumed to be main or include main
)
ament_target_dependencies(waypoint_generator_node PUBLIC
  rclcpp
  safe_landing_planner # Links against the main library (which includes waypoint_generator logic)
)
# target_link_libraries(waypoint_generator_node PUBLIC safe_landing_planner) # Alternative way


  #############
  ## Install ##
  #############
  install(TARGETS safe_landing_planner safe_landing_planner_node waypoint_generator_node
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME} # Install executables
  )

  install(DIRECTORY include/
    DESTINATION include/
    FILES_MATCHING PATTERN "*.h"
  )

  ament_export_include_directories(include)
  ament_export_libraries(safe_landing_planner)
  ament_export_dependencies(
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    mav_msgs
    tf2_ros
    pcl_conversions
    mavros
    mavros_msgs
    avoidance
    rosidl_default_runtime # For SLPGridMsg
    # yaml-cpp is not typically exported as a ROS dep
  )

  install(EXPORT export_${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
  )

  # Install parameter files (converted from .cfg files)
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

    set(SLP_TEST_FILES test/main.cpp
                       test/test_safe_landing_planner.cpp
                       test/test_waypoint_generator.cpp
                       test/test_grid.cpp
    )
    ament_add_gtest(${PROJECT_NAME}-test "${SLP_TEST_FILES}")

    target_link_libraries(${PROJECT_NAME}-test PRIVATE
      safe_landing_planner # Link against the library being tested
      ${PCL_LIBRARIES}
      ${yaml-cpp_LIBRARIES} # If tests use yaml-cpp directly or through lib
    )
    ament_target_dependencies(${PROJECT_NAME}-test PUBLIC
      rclcpp # For ROS utilities in tests
      # Add other specific ROS dependencies if test code uses them directly
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
            COMMAND ${CMAKE_CTEST_COMMAND} --force-new-ctest-process --output-on-failure # Run all tests
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
