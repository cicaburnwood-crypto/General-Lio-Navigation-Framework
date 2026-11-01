cmake_minimum_required(VERSION 3.8)
project(livox_self_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

add_library(livox_self_filter_core src/self_filter.cpp)
target_include_directories(livox_self_filter_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(livox_self_filter_core
  livox_ros_driver2
  tf2
)

add_executable(livox_self_filter_node src/livox_self_filter_node.cpp)
target_link_libraries(livox_self_filter_node livox_self_filter_core)
ament_target_dependencies(livox_self_filter_node
  livox_ros_driver2
  rclcpp
  tf2
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS livox_self_filter_core livox_self_filter_node
  EXPORT export_livox_self_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_self_filter test/test_self_filter.cpp)
  if(TARGET test_self_filter)
    target_link_libraries(test_self_filter livox_self_filter_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_livox_self_filter HAS_LIBRARY_TARGET)
ament_export_dependencies(livox_ros_driver2 tf2)
ament_package()
