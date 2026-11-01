cmake_minimum_required(VERSION 3.8)
project(ply_map_converter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)

add_executable(ply_map_converter src/ply_map_converter.cpp)

target_include_directories(ply_map_converter
  PRIVATE
    ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(ply_map_converter PRIVATE ${PCL_DEFINITIONS})

target_link_libraries(ply_map_converter
  ${PCL_LIBRARIES}
)

install(TARGETS ply_map_converter DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/align_centered_ply_to_floor.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
