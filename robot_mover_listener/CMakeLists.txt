cmake_minimum_required(VERSION 3.0.2)
project(robot_mover_listener)

## Compile as C++11
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
#  sensor_msgs
  std_msgs
  message_generation
#  geometry_msgs
#  pcl_ros      # Include PCL ROS wrapper
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

## Add executable
add_executable(robot_mover src/robot_mover.cpp src/iRobot_Create/Roomba.cpp src/iRobot_Create/iCreateDef.cpp)

target_include_directories(robot_mover PRIVATE ./src/iRobot_Create)


## Specify libraries to link against
target_link_libraries(robot_mover ${catkin_LIBRARIES}
        #k4a #kinect azure stuff
        #k4abt
        #k4arecord
        #window_controller_3d::window_controller_3d
        #glfw::glfw
)
