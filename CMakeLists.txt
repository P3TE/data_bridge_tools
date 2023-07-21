cmake_minimum_required(VERSION 3.0.2)

project(data_bridge_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp
)

add_executable(ros_to_serial_bridge src/serial/ros_to_serial_bridge.cpp)
target_link_libraries(ros_to_serial_bridge ${catkin_LIBRARIES})
