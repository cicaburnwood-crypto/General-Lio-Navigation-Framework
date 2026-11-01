cmake_minimum_required(VERSION 3.8)
project(agv_mpc_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_core
  src/sampling_mpc.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME} SHARED
  src/mpc_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core agv_mpc_controller.xml)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES agv_mpc_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_sampling_mpc test/test_sampling_mpc.cpp)
  if(TARGET test_sampling_mpc)
    target_link_libraries(test_sampling_mpc ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_speed_limit test/test_speed_limit.cpp)
  if(TARGET test_speed_limit)
    target_include_directories(test_speed_limit PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(test_speed_limit ${PROJECT_NAME}_core)
  endif()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
ament_package()
