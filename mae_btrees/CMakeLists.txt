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
    mae_utils
    
    )

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )


catkin_package(
    INCLUDE_DIRS include src/libraries
    LIBRARIES
    CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories( include src/libraries ${catkin_INCLUDE_DIRS})
install(DIRECTORY trees DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

######################################################
# LIBRARIES

#header only for the time being

#add_library(behaviortree_ros ... )
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

######################################################
# TESTS

add_library(bt_actions src/libraries/actions.cpp )
add_library(bt_conditions src/libraries/conditions.cpp)
add_library(handle_bt src/HandleBT.cpp)
add_library(ros_comm src/libraries/ros_comm.cpp)
add_library(bt_utils src/libraries/utils.cpp)




add_executable(bt_node src/behavior_tree_node.cpp)
add_dependencies(bt_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bt_node 
                        handle_bt
                        ros_comm
                        bt_actions
                        bt_utils
                        bt_conditions
                        ${catkin_LIBRARIES})



######################################################
# INSTALL


######################################################
# EXAMPLES and TOOLS



