cmake_minimum_required(VERSION 3.8)
project(pose_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
  include
  ${ARMADILLO_INCLUDE_DIRS}
)

add_library(pose_utils
  src/pose_utils.cpp
)
ament_target_dependencies(pose_utils rclcpp)
target_link_libraries(pose_utils ${ARMADILLO_LIBRARIES})

install(TARGETS pose_utils
  EXPORT export_pose_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(pose_utils)
ament_export_dependencies(rclcpp Armadillo)

ament_package()
