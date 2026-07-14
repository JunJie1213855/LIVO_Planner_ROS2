cmake_minimum_required(VERSION 3.8)
project(mockamap)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# NOTE: src/ces_randommap.cpp is legacy/historical code (no main(), functions
# never called by the node) and is intentionally excluded from the build.
add_executable(mockamap_node
  src/mockamap.cpp
  src/maps.cpp
  src/perlinnoise.cpp
)
ament_target_dependencies(mockamap_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)
target_link_libraries(mockamap_node ${PCL_LIBRARIES})

install(TARGETS mockamap_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
