cmake_minimum_required(VERSION 3.0.2)
project(kinect_azure_publisher)

## Compile as C++11
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  pcl_ros      # Include PCL ROS wrapper
  pcl_conversions # Include PCL conversions
)

# Find PCL
find_package(PCL REQUIRED)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add executable
add_executable(kinect_publisher src/kinect_publisher.cpp)

target_include_directories(kinect_publisher PRIVATE ./src/sample_helper_includes)

## Specify libraries to link against
target_link_libraries(kinect_publisher ${catkin_LIBRARIES}
        k4a #kinect azure stuff
        k4abt
        k4arecord
        #window_controller_3d::window_controller_3d
        #glfw::glfw
)
