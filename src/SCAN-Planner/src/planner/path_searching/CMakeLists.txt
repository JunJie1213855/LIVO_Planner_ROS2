cmake_minimum_required(VERSION 3.8)
project(path_searching)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(plan_env REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PATH_SEARCHING_DEPS
  rclcpp
  std_msgs
  visualization_msgs
  cv_bridge
  plan_env
)

add_library(path_searching
  src/dyn_a_star.cpp
)
ament_target_dependencies(path_searching ${PATH_SEARCHING_DEPS})
target_link_libraries(path_searching
  ${PCL_LIBRARIES}
)

install(TARGETS path_searching
  EXPORT export_path_searching
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(path_searching)
ament_export_dependencies(${PATH_SEARCHING_DEPS} Eigen3 PCL)

ament_package()
