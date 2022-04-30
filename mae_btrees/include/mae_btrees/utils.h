#ifndef UTILS_H
#define UTILS_H
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

template <>
geometry_msgs::Pose BT::convertFromString(StringView str);

float calcDistPoses(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);
#endif // UTILS_H