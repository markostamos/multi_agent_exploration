cmake_minimum_required(VERSION 3.5.1) # version on Ubuntu Trusty
project(mae_frontier_generation)


######################################################

set(ROS_DEPENDENCIES
    roscpp 
    std_msgs
    geometry_msgs
    )

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )
find_package(OpenCV)
  
   
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories( include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
######################################################
# LIBRARIES

#header only for the time being

#add_library(behaviortree_ros ... )
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

######################################################
# TESTS




add_executable(frontier_generation src/frontier_generation.cpp)
add_dependencies(frontier_generation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(frontier_generation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
