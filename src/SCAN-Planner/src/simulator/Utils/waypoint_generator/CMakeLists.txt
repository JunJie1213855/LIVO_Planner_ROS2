cmake_minimum_required(VERSION 3.8)
project(waypoint_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(waypoint_generator src/waypoint_generator.cpp)
ament_target_dependencies(waypoint_generator
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS waypoint_generator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
