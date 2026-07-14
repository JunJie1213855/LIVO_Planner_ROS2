cmake_minimum_required(VERSION 3.8)
project(local_sensing_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

option(USE_GPU "Build GPU OpenGL renderer" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ---- CPU renderer (default) ----
add_executable(pcl_render_node
  src/pointcloud_render_node.cpp
  include/ikd-Tree/ikd_Tree.cpp
  include/FOV_Checker/FOV_Checker.cpp
)
ament_target_dependencies(pcl_render_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
  tf2_ros
  ament_index_cpp
)
target_link_libraries(pcl_render_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
  Threads::Threads
)

install(TARGETS pcl_render_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

# ---- GPU OpenGL renderer ----
# TODO Phase B (GPU): opengl_render_node is deferred. Enable with -DUSE_GPU=ON once
# GLEW/OpenGL/glfw/GLAD are provisioned on the target machine.
if(USE_GPU)
  add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

  find_package(GLEW REQUIRED)
  find_package(OpenGL REQUIRED)
  find_package(glfw3 QUIET)
  if(NOT glfw3_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw/usr")
    set(glfw3_FOUND TRUE)
    set(glfw3_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw/usr/include")
    set(glfw3_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw/usr/lib/x86_64-linux-gnu/libglfw.so.3.3")
  endif()
  if(NOT glfw3_FOUND)
    message(FATAL_ERROR "USE_GPU=ON requires glfw3. Install libglfw3-dev or provide third_party/glfw.")
  endif()
  if(NOT glfw3_LIBS)
    set(glfw3_LIBS glfw)
  endif()

  include_directories(
    ${OPENGL_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIRS}
    ${glfw3_INCLUDE_DIRS}
  )

  add_library(GLAD include/glad.c)

  add_executable(opengl_render_node
    src/opengl_render_node.cpp
    include/ikd-Tree/ikd_Tree.cpp
    include/FOV_Checker/FOV_Checker.cpp
  )
  ament_target_dependencies(opengl_render_node
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
    pcl_ros
    pcl_conversions
    tf2_ros
    ament_index_cpp
  )
  target_link_libraries(opengl_render_node
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${glfw3_LIBS}
    GLAD
    OpenMP::OpenMP_CXX
    Threads::Threads
  )
  install(TARGETS opengl_render_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "USE_GPU=OFF: skipping opengl_render_node")
endif()

ament_package()
