cmake_minimum_required(VERSION 3.5.1) # version on Ubuntu Trusty
project(mae_btrees)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

######################################################

set(ROS_DEPENDENCIES
    roscpp std_msgs
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
add_library(test_actions_library src/nodes/test_bt_actions.cpp) 
target_link_libraries(test_actions_library ${catkin_LIBRARIES})

add_library(utils src/nodes/utils.cpp) 
target_link_libraries(utils ${catkin_LIBRARIES})

add_library(drone_actions src/nodes/drone_actions.cpp) 
target_link_libraries(drone_actions ${catkin_LIBRARIES})



add_executable(bt src/test_bt.cpp)
add_dependencies(bt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bt ${catkin_LIBRARIES} test_actions_library utils drone_actions )



######################################################
# INSTALL


######################################################
# EXAMPLES and TOOLS



