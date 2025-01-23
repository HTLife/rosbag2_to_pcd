cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)
project(bag2pcd)

## Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

## Include directories
include_directories(
  include
  SYSTEM ${PCL_INCLUDE_DIRS}
)

## Add library for Rosbag2ToPcdNode
add_library(rosbag2_to_pcd_node
  src/rosbag2_to_pcd.cpp
)

ament_target_dependencies(rosbag2_to_pcd_node
  rclcpp
  sensor_msgs
  rosbag2_cpp
  pcl_conversions
  PCL
)

target_include_directories(rosbag2_to_pcd_node PRIVATE include)

## Add executable
add_executable(bag2pcd src/main.cpp)
target_link_libraries(bag2pcd rosbag2_to_pcd_node)

## Install targets
install(
  TARGETS bag2pcd rosbag2_to_pcd_node
  DESTINATION lib/${PROJECT_NAME}
)

## Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Package
ament_package()
