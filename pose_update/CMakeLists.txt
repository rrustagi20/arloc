cmake_minimum_required(VERSION 3.0.2)
project(pose_update)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  eigen_conversions
  roscpp
  sensor_msgs
  std_msgs
  # tf2_msgs
  # tf
  # tf2
  tf2_ros
  apriltag_ros
  amcl
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   pose_struct.msg
#   pose_array.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
#   std_msgs
#   geometry_msgs
# )

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES detection
 CATKIN_DEPENDS roscpp 
 sensor_msgs std_msgs message_runtime  
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(amcl_lib src/amcl_sampling.cpp)

# add_executable(camera src/camera.cpp)
# target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBS})

# add_executable(camera_info src/caminfo.cpp)
# target_link_libraries(camera_info ${catkin_LIBRARIES} ${OpenCV_LIBS})

# add_executable(april src/april_pose.cpp)
# target_link_libraries(april ${catkin_LIBRARIES} ${OpenCV_LIBS} apriltag::apriltag)#${apriltag_LIBRARIES})

# add_executable(tracker src/tracker.cpp)
# target_link_libraries(tracker ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} )

# add_executable(opttracker src/optTracker.cpp)
# target_link_libraries(opttracker ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} apriltag::apriltag)

# add_executable(kalman_node src/kalman.cpp)
# target_link_libraries(kalman_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

# add_executable(amcl_node src/amcl_node.cpp)
# target_link_libraries(amcl_node amcl_lib ${catkin_LIBRARIES})

add_executable(pose_fuse src/pose_fuse.cpp)
target_link_libraries(pose_fuse ${catkin_LIBRARIES})

add_executable(pose_new src/poseNew.cpp)
target_link_libraries(pose_new ${catkin_LIBRARIES})