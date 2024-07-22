cmake_minimum_required(VERSION 3.0.2)
project(marker_pose)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  geometry_msgs
  roscpp
  rospy
  tf2_msgs
  tf
  tf2
  tf2_ros
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES marker_pose
 CATKIN_DEPENDS geometry_msgs roscpp rospy tf2
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
## add_library(${PROJECT_NAME}
##   src/${PROJECT_NAME}/marker_pose.cpp
## )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
## add_executable(${PROJECT_NAME}_io_pose_node src/io_pose.cpp)
## add_executable(${PROJECT_NAME}_tf2_testing_node src/tf2Testing.cpp)
## add_executable(${PROJECT_NAME}_tf2_asd_node src/tf2asd.cpp)
## add_executable(${PROJECT_NAME}_merge_pose_node src/merge_pose.cpp)

add_executable(${PROJECT_NAME}_abs_pose_calc src/abs_pose_calc.cpp)

## Specify libraries to link a library or executable target against
##target_link_libraries(${PROJECT_NAME}_io_pose_node
##  ${catkin_LIBRARIES}
##)
##target_link_libraries(${PROJECT_NAME}_tf2_testing_node
##  ${catkin_LIBRARIES}
##)
##target_link_libraries(${PROJECT_NAME}_tf2_asd_node
##  ${catkin_LIBRARIES}
##)
##target_link_libraries(${PROJECT_NAME}_merge_pose_node
##  ${catkin_LIBRARIES}
##)

target_link_libraries(${PROJECT_NAME}_abs_pose_calc
  ${catkin_LIBRARIES}
)