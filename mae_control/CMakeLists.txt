cmake_minimum_required(VERSION 3.0.2)
project(mae_control)

set(ROS_DEPENDENCIES
    rospy
    tf
    move_base
    roscpp
    std_msgs 
    nav_msgs
    geometry_msgs
)

find_package(ompl REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES control
   CATKIN_DEPENDS ${ROS_DEPENDENCIES}
   DEPENDS system_lib ompl
)



catkin_install_python(PROGRAMS 
  src/rviz_goal_publisher_node.py
  src/vel_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(path_planner_3d_node src/path_planner_3d_node.cpp)
target_link_libraries(path_planner_3d_node ${catkin_LIBRARIES})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
