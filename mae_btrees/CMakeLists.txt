cmake_minimum_required(VERSION 3.5.1) # version on Ubuntu Trusty
project(mae_btrees)


######################################################

set(ROS_DEPENDENCIES
    roscpp 
    std_msgs
    geometry_msgs
    behaviortree_cpp_v3
    actionlib_msgs
    actionlib
    
    )

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories( include ${catkin_INCLUDE_DIRS})
install(DIRECTORY trees DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

######################################################
# LIBRARIES

#header only for the time being

#add_library(behaviortree_ros ... )
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

######################################################
# TESTS




add_executable(bt src/test_bt.cpp)
add_dependencies(bt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bt ${catkin_LIBRARIES})



######################################################
# INSTALL


######################################################
# EXAMPLES and TOOLS



