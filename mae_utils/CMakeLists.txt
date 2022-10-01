cmake_minimum_required(VERSION 3.0.2)
project(mae_utils)

set(ROS_DEPENDENCIES
  rospy
  roscpp
  geometry_msgs
  tf
  std_msgs
  message_generation  
)
find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )

add_message_files(
  FILES
  PointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utils_lib
  CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(utils_lib src/utils.cpp)
target_link_libraries(utils_lib ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS 
  src/global_communication_node.py
  src/drone_comm_node.py
  src/gui_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

