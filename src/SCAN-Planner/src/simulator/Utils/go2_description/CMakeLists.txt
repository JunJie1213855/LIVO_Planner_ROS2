cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes dae xacro config launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
