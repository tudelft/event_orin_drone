cmake_minimum_required(VERSION 3.8)
project(depth_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(depth_to_bins_component SHARED src/depth_to_bins.cpp)
ament_target_dependencies(depth_to_bins_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  OpenCV
  cv_bridge
)

rclcpp_components_register_nodes(depth_to_bins_component "DepthToBinsNode")

install(TARGETS
  depth_to_bins_component
  DESTINATION lib)

ament_package()
