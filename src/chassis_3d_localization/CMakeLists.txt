cmake_minimum_required(VERSION 3.8)
project(chassis_3d_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(chassis_localization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scan_registration REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(chassis_3d_localizer_node src/chassis_3d_localizer_node.cpp)

target_include_directories(chassis_3d_localizer_node
  PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(chassis_3d_localizer_node
  chassis_localization_msgs
  geometry_msgs
  message_filters
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2_ros
)

target_link_libraries(chassis_3d_localizer_node
  ${PCL_LIBRARIES}
  scan_registration::scan_registration
)

install(TARGETS chassis_3d_localizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_alignment_gate test/test_alignment_gate.cpp)
  target_include_directories(test_alignment_gate PRIVATE include)

  ament_add_gtest(test_score_gate test/test_score_gate.cpp)
  target_include_directories(test_score_gate PRIVATE include)

  ament_add_gtest(test_correction_smoothing test/test_correction_smoothing.cpp)
  target_include_directories(test_correction_smoothing
    PRIVATE include ${EIGEN3_INCLUDE_DIR})
endif()

ament_package()
