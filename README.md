# edu-franka_simulator_ros2
Franka Panda simulation for AUT-720 Advanced Robotics
We'll use [[4]](https://github.com/modulabs/arm-control) for implementing and demonstrating the course assignments. 

## Prerequisites
It is recommended to use the following requirements to avoid any unforseen error during the implementation of the codes.

- Ubuntu 22.04
- ROS2 Humble - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html


## (Optional) VMware:
Suboptimal performance in Gazebo simulations due to only CPU rendering available. 
IMPORTANT: Following arqument required when launching simulations in WMware

```
LIBGL_ALWAYS_SOFTWARE=1
```
Example:
```
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch elfin_gazebo elfin.launch.py
```

## Installation
### Install effort-controllers to use torque-control interface
```
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-publisher-gui ros-humble-ros-gz
```
### Install gazebo-ros-pkgs and gazebo-ros-control
```
sudo apt-get install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-effort-controllers
```
### Download, create a workspace and build 
    $ mkdir -p elfin_ros2_ws/src
    $ cd ~/elfin_ros2_ws/src
    $ git clone https://github.com/JormaKuusYsi/edu-franka_simulator_ros2.git
    $ cd ~/elfin_ros2_ws/
    $ colcon build
    $ source devel/setup.bash

## Launch
Open new terminal:
```
cd elfin_ros2_ws/
colcon build
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:#PATH_TO_ELFIN_FOLDER
```
i.e. 
```
export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:/home/user/elfin_ros2_ws/src/elfin
```
```
ros2 launch elfin_gazebo elfin.launch.py
```
