cmake_minimum_required(VERSION 3.8)
project(traj_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline_opt REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(TRAJ_UTILS_DEPS
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  plan_env
  path_searching
  bspline_opt
)

add_library(traj_utils
  src/planning_visualization.cpp
  src/polynomial_traj.cpp
)
ament_target_dependencies(traj_utils ${TRAJ_UTILS_DEPS})
target_link_libraries(traj_utils
  ${PCL_LIBRARIES}
)

install(TARGETS traj_utils
  EXPORT export_traj_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(traj_utils)
ament_export_dependencies(${TRAJ_UTILS_DEPS} Eigen3 PCL)

ament_package()
