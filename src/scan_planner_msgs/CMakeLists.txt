cmake_minimum_required(VERSION 3.8)
project(scan_planner_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  "msg/DataDisp.msg"
  "msg/NavigationStatus.msg"
  "msg/GlobalPlanRequest.msg"
  "msg/GlobalPlanResult.msg"
  "msg/TaskPath.msg"
  "srv/ActivateTask.srv"
  "srv/StopTask.srv"
  "srv/DetectDoorOpening.srv"
  "action/NavigateTo.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs nav_msgs unique_identifier_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
