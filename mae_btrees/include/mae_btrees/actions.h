#ifndef ACTIONS_H
#define ACTIONS_H

// Libraries used by all actions
#include <ros/ros.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

// include utils
#include <mae_btrees/ros_comm.h>
#include <mae_btrees/utils.h>

// include all actions
#include <actions/GoTo.h>
#include <actions/TakeOff.h>
#include <actions/Land.h>
#include <actions/SetLocation.h>

#endif // ACTIONS_H