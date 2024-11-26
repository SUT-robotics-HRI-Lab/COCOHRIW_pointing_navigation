cmake_minimum_required(VERSION 3.0.2)
project(mozek_decider)

## Compile as C++11
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  tf
#  geometry_msgs
  pcl_ros      # Include PCL ROS wrapper
#  pcl_conversions # Include PCL conversions
)

add_message_files(
  FILES
  AngleDistance.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(decider src/decider.cpp)
target_link_libraries(decider ${catkin_LIBRARIES})




