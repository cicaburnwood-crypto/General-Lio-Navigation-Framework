cmake_minimum_required(VERSION 3.8)
project(nav_goal_regenerator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(goal_projector src/goal_projector.cpp)
target_include_directories(goal_projector PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_library(table_goal_adapter src/table_goal_adapter.cpp)
target_include_directories(table_goal_adapter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(table_goal_adapter PUBLIC goal_projector)

add_library(direct_motion src/direct_motion.cpp)
target_include_directories(direct_motion PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_library(arrival_monitor_state INTERFACE)
target_include_directories(arrival_monitor_state INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_executable(goal_regenerator_node src/goal_regenerator_node.cpp)
target_link_libraries(goal_regenerator_node goal_projector)
ament_target_dependencies(goal_regenerator_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(task_navigation_bridge_node src/task_navigation_bridge_node.cpp)
target_link_libraries(task_navigation_bridge_node goal_projector table_goal_adapter direct_motion)
ament_target_dependencies(task_navigation_bridge_node
  geometry_msgs
  nav_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(arrival_monitor_node src/arrival_monitor_node.cpp)
target_link_libraries(arrival_monitor_node arrival_monitor_state)
ament_target_dependencies(arrival_monitor_node
  action_msgs
  geometry_msgs
  rclcpp
  std_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS goal_projector table_goal_adapter direct_motion
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS goal_regenerator_node task_navigation_bridge_node arrival_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_goal_projector test/test_goal_projector.cpp)
  target_link_libraries(test_goal_projector goal_projector)

  ament_add_gtest(test_table_goal_adapter test/test_table_goal_adapter.cpp)
  target_link_libraries(test_table_goal_adapter table_goal_adapter)

  ament_add_gtest(test_direct_motion test/test_direct_motion.cpp)
  target_link_libraries(test_direct_motion direct_motion)

  ament_add_gtest(test_arrival_monitor_state test/test_arrival_monitor_state.cpp)
  target_link_libraries(test_arrival_monitor_state arrival_monitor_state)
endif()

ament_package()
