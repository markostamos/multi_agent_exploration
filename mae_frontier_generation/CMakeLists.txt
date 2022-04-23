cmake_minimum_required(VERSION 3.5.1)
project(mae_frontier_generation)


######################################################

set(ROS_DEPENDENCIES
    roscpp 
    nav_msgs
    geometry_msgs
    )

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES frontier_generation utils
    CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_library(frontier_generation src/FrontierGeneration.cpp)
add_library(fg_utils src/utils.cpp)




add_executable(frontier_generation_node src/frontier_generation_node.cpp)
add_dependencies(frontier_generation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(frontier_generation_node 
                        fg_utils 
                        frontier_generation 
                        ${catkin_LIBRARIES})
