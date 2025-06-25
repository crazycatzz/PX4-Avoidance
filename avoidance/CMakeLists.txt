cmake_minimum_required(VERSION 3.16)
project(avoidance)

# Standard C++17
# set(CMAKE_CXX_STANDARD 17) # ament_cmake_auto_set_compiler_flags usually handles this
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find ament_cmake and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED) # If python nodes/scripts are added
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED) # For converting PCL types to ROS messages
find_package(PCL REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(yaml-cpp REQUIRED) # System dependency

# The DISABLE_SIMULATION logic will be handled by conditionally compiling rviz_world_loader.cpp

option(DISABLE_SIMULATION "Disable Gazebo simulation features" OFF) # Moved to top
if(DISABLE_SIMULATION)
  message(STATUS "Building avoidance without Gazebo Simulation features (rviz_world_loader.cpp)")
  add_compile_definitions(DISABLE_SIMULATION) # Keep preprocessor flag if C++ code uses it
else()
  message(STATUS "Building avoidance with Gazebo Simulation features (rviz_world_loader.cpp)")
endif()

ament_package() # ROS 2 replacement for catkin_package()

###########
## Build ##
###########
ament_cmake_auto_set_compiler_flags()

## CMake Setup
# Build in Release mode if nothing is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

## Declare a C++ library
set(AVOIDANCE_CPP_FILES   "src/common.cpp"
                          "src/histogram.cpp"
                          "src/transform_buffer.cpp"
                          "src/avoidance_node.cpp"
)
if(NOT DISABLE_SIMULATION)
  list(APPEND AVOIDANCE_CPP_FILES "src/rviz_world_loader.cpp")
endif()
add_library(avoidance SHARED "${AVOIDANCE_CPP_FILES}") # Changed to SHARED for typical ROS library

## Specify include directories for the library
target_include_directories(avoidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS} # Assuming find_package(yaml-cpp) provides this
)

## Specify libraries to link a library or executable target against
ament_target_dependencies(avoidance PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  mavros
  mavros_msgs
)

target_link_libraries(avoidance PUBLIC
  ${PCL_LIBRARIES}
  ${yaml-cpp_LIBRARIES} # Assuming find_package(yaml-cpp) provides this
)

#############
## Install ##
#############
    install(TARGETS avoidance
      EXPORT avoidance_targets
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin # For executables, not typical for SHARED libraries
    )

    # Install headers
    install(DIRECTORY include/
      DESTINATION include/
      FILES_MATCHING PATTERN "*.h" # Ensure only headers are installed
    )

    # Export include directories and libraries for downstream packages
    ament_export_include_directories(include)
    ament_export_libraries(avoidance)

    # Install CMake config files (for find_package)
    install(EXPORT avoidance_targets
      NAMESPACE ${PROJECT_NAME}:: # Or your chosen namespace
      DESTINATION lib/cmake/${PROJECT_NAME}
    )

#############
## Testing ##
#############

    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      # Add linters and gtest/gmock dependencies
      ament_lint_auto_find_test_dependencies()

      set(AVOIDANCE_TEST_FILES test/main.cpp
                               test/test_common.cpp
                               test/test_usm.cpp
                               test/test_transform_buffer.cpp
      )
      ament_add_gtest(${PROJECT_NAME}-test "${AVOIDANCE_TEST_FILES}")

      # Add include directories for the test executable
      target_include_directories(${PROJECT_NAME}-test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include # To find project headers
        # ${PCL_INCLUDE_DIRS} # Already linked via PCL_LIBRARIES usually
        # ${yaml-cpp_INCLUDE_DIRS} # Already linked via yaml-cpp_LIBRARIES usually
      )

      # Link the test executable against the library and other dependencies
      # ament_target_dependencies can also be used here for ROS packages
      target_link_libraries(${PROJECT_NAME}-test PRIVATE
        avoidance # Link against the library being tested
        ${PCL_LIBRARIES}
        ${yaml-cpp_LIBRARIES}
      )
      # Ensure ROS dependencies are available to the test target
      ament_target_dependencies(${PROJECT_NAME}-test PUBLIC
        rclcpp # Or other specific ROS deps needed by tests
      )


      if (${CMAKE_BUILD_TYPE} STREQUAL "Coverage")
          SET(CMAKE_CXX_FLAGS_INIT "${CMAKE_CXX_FLAGS}" CACHE STRING "" FORCE) # Store original flags
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
              DEPENDS ${PROJECT_NAME}-test # Depends on the test target having been built
          )
          add_custom_target(${PROJECT_NAME}-test_coverage_html
              COMMAND genhtml coverage.info --output-directory out --branch-coverage
              # COMMAND x-www-browser out/index.html # Removed interactive command
              WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
              DEPENDS ${PROJECT_NAME}-test_coverage
          )
      endif()
endif()
