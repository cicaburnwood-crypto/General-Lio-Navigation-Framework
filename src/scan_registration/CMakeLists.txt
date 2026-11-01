cmake_minimum_required(VERSION 3.8)
project(scan_registration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

add_library(${PROJECT_NAME} SHARED
  src/registration_methods.cpp
  src/fast_gicp/gicp/fast_gicp.cpp
  src/fast_gicp/gicp/fast_vgicp.cpp
  src/fast_gicp/gicp/lsq_registration.cpp
  src/pclomp/gicp_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/voxel_grid_covariance_omp.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC ${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${PCL_LIBRARIES}
)

if(OpenMP_FOUND AND TARGET OpenMP::OpenMP_CXX)
  target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 PCL)
if(OpenMP_FOUND AND TARGET OpenMP::OpenMP_CXX)
  ament_export_dependencies(OpenMP)
endif()
ament_package()
