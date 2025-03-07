cmake_minimum_required(VERSION 3.8)
project(eo_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS})
add_executable(fly_square src/fly_square.cpp)
ament_target_dependencies(fly_square Eigen3 px4_ros2_cpp rclcpp)
add_executable(depth_seeker src/depth_seeker.cpp)
ament_target_dependencies(depth_seeker std_msgs Eigen3 px4_ros2_cpp rclcpp)

install(TARGETS
  fly_square
  depth_seeker
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
