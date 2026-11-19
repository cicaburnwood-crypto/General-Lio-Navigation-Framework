cmake_minimum_required(VERSION 3.8)
project(nav_interface_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(nav_interface_adapter_node src/nav_interface_adapter_node.cpp)
target_include_directories(nav_interface_adapter_node PRIVATE include)
ament_target_dependencies(nav_interface_adapter_node
  geometry_msgs
  livox_ros_driver2
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

install(TARGETS nav_interface_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_obstacle_height_filter test/test_obstacle_height_filter.cpp)
  target_include_directories(test_obstacle_height_filter PRIVATE include)
  ament_target_dependencies(test_obstacle_height_filter sensor_msgs tf2)
endif()

ament_package()
