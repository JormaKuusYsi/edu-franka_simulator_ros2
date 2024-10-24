cmake_minimum_required(VERSION 3.5)

project(elfin_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ros_gz REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(elfin_description REQUIRED)

# Install project launch files
install(
  DIRECTORY
    launch config
  DESTINATION share/${PROJECT_NAME}
)

  
ament_package()
