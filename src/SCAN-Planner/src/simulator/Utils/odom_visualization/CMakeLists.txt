cmake_minimum_required(VERSION 3.8)
project(odom_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pose_utils REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(${ARMADILLO_INCLUDE_DIRS})

add_executable(odom_visualization src/odom_visualization.cpp)
ament_target_dependencies(odom_visualization
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  tf2_ros
  pose_utils
)
target_link_libraries(odom_visualization ${ARMADILLO_LIBRARIES})

install(TARGETS odom_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
