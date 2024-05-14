cmake_minimum_required(VERSION 3.0.2)
project(sriod_scout)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Add message files
add_message_files(
  FILES
  SriodData.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)



## Declare a C++ executable
add_executable(sriod_scout_node scripts/sriod_scouting.cpp)

## Specify libraries to link against
target_link_libraries(sriod_scout_node
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS sriod_scout_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Python scripts
catkin_install_python(
  PROGRAMS
    scripts/SriodNavigation.py  # Add your Python script here
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


