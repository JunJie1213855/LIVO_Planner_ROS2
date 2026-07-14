cmake_minimum_required(VERSION 3.8)
project(map_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(map_pub src/map_publisher.cpp)
ament_target_dependencies(map_pub
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(map_pub ${PCL_LIBRARIES})

install(TARGETS map_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
