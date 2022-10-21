cmake_minimum_required(VERSION 3.0.2)
project(mae_control)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(ROS_DEPENDENCIES
    rospy
    tf
    move_base
    roscpp
    std_msgs 
    nav_msgs
    geometry_msgs
    octomap_msgs
    genmsg
    actionlib_msgs
    actionlib
)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )
find_package(octomap REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization)
find_package(Eigen3 REQUIRED)
find_package(ompl)



add_action_files(
  DIRECTORY action
  FILES
  MoveBase3D.action
)
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)



include_directories(
 include
   ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES control
   CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)



catkin_install_python(PROGRAMS 
  src/rviz_goal_publisher_node.py
  src/vel_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(planner_3d src/Planner3D.cpp)
target_link_libraries(planner_3d ompl ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(move_base_3d_node src/move_base_3d_node.cpp)
target_link_libraries(move_base_3d_node 
                     ${catkin_LIBRARIES}
                     ${OCTOMAP_LIBRARIES}
                     planner_3d
                     ompl)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
