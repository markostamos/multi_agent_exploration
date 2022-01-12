cmake_minimum_required(VERSION 3.0.2)
project(mae_utils)


find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  
)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS 
  src/base_footprint_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
