#ifndef CONDITIONS_H
#define CONDITIONS_H

// Libraries used by all actions
#include <ros/ros.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

// include utils
#include <mae_btrees/ros_comm.h>
#include <mae_btrees/utils.h>
#include <mae_utils/utils.h>

// include all actions
#include <conditions/isFrontierListEmpty.h>
#include <conditions/TargetDiscovered.h>
#include <conditions/GreedyTargetDiscovered.h>
#include <conditions/isNearOtherDrones.h>
#include <conditions/NewTaskReceived.h>
#include <conditions/LowBattery.h>

#endif // CONDITIONS_H