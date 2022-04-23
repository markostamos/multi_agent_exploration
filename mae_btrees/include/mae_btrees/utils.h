#ifndef UTILS_H
#define UTILS_H
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

template <>
geometry_msgs::Pose BT::convertFromString(StringView str);

geometry_msgs::Pose poseMsgFromVec(std::vector<double> pos, std::vector<double> rpy = {0, 0, 0});

geometry_msgs::PoseStamped poseStampedMsgFromVec(std::vector<double> pos, std::vector<double> rpy = {0, 0, 0});

geometry_msgs::Twist twistMsgFromVec(std::vector<double> linear = {0, 0, 0}, std::vector<double> angular = {0, 0, 0});

#endif // UTILS_H