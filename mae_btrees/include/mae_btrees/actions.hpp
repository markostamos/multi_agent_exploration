#ifndef ACTIONS_H
#define ACTIONS_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

// include utils
#include <mae_btrees/ros_comm.hpp>
#include <mae_btrees/utils.hpp>

// include all actions
#include <mae_btrees/actions/GoTo.hpp>
#include <mae_btrees/actions/TakeOff.hpp>
#include <mae_btrees/actions/Land.hpp>
#include <mae_btrees/actions/SetLocation.hpp>

// common msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#endif // ACTIONS_H