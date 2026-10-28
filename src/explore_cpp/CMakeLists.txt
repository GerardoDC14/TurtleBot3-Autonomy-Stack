cmake_minimum_required(VERSION 3.8)
project(explore_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(explore_node src/explore_node.cpp)
target_compile_features(explore_node PRIVATE cxx_std_17)
ament_target_dependencies(explore_node
  rclcpp rclcpp_action nav_msgs geometry_msgs visualization_msgs
  tf2_ros tf2_geometry_msgs nav2_msgs)
install(TARGETS explore_node DESTINATION lib/${PROJECT_NAME})

ament_package()
