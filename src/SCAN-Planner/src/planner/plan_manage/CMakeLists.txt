cmake_minimum_required(VERSION 3.8)
project(scan_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(scan_planner_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SCAN_PLANNER_DEPS
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  cv_bridge
  plan_env
  path_searching
  bspline_opt
  traj_utils
  scan_planner_msgs
)

add_executable(scan_planner_node
  src/scan_planner_node.cpp
  src/scan_replan_fsm.cpp
  src/planner_manager.cpp
)
ament_target_dependencies(scan_planner_node ${SCAN_PLANNER_DEPS})
target_link_libraries(scan_planner_node ${PCL_LIBRARIES})

add_executable(open_loop_controller src/open_loop_controller.cpp)
ament_target_dependencies(open_loop_controller ${SCAN_PLANNER_DEPS})
target_link_libraries(open_loop_controller ${PCL_LIBRARIES})

add_executable(closed_loop_controller src/closed_loop_controller.cpp)
ament_target_dependencies(closed_loop_controller ${SCAN_PLANNER_DEPS})
target_link_libraries(closed_loop_controller ${PCL_LIBRARIES})

add_executable(go2_kinematic_sim src/go2_kinematic_sim.cpp)
ament_target_dependencies(go2_kinematic_sim ${SCAN_PLANNER_DEPS})
target_link_libraries(go2_kinematic_sim ${PCL_LIBRARIES})

add_executable(go2_gait_publisher src/go2_gait_publisher.cpp)
ament_target_dependencies(go2_gait_publisher ${SCAN_PLANNER_DEPS})
target_link_libraries(go2_gait_publisher ${PCL_LIBRARIES})

install(TARGETS
  scan_planner_node
  open_loop_controller
  closed_loop_controller
  go2_kinematic_sim
  go2_gait_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# RViz2 example config lives at the repo root (SCAN-Planner/rviz/); install it
# into the package share so launch files can find it via FindPackageShare.
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../rviz/example.rviz
  DESTINATION share/${PROJECT_NAME}/rviz
  OPTIONAL
)

ament_package()
