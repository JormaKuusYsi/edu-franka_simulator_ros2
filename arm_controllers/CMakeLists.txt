cmake_minimum_required(VERSION 3.5)

project(arm_controllers)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(controller_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(control_msgs REQUIRED)

install(DIRECTORY cfg include msg src
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
