#ifndef MAE_UTILS_H
#define MAE_UTILS_H
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <mae_utils/PointArray.h>
#include <tf/LinearMath/Quaternion.h>
#include <ros/ros.h>
#define logit_(x) ROS_WARN_STREAM(#x << ": " << x)
#define log_(x) ROS_WARN_STREAM(x)
#define logmany_(x)  \
    for (auto i : x) \
    ROS_WARN_STREAM(i)

geometry_msgs::Pose poseFrom2DMapIndex(const int i, const int j, const nav_msgs::OccupancyGrid &map);
geometry_msgs::Point pointFrom2DMapIndex(const int i, const int j, const nav_msgs::OccupancyGrid &map);
visualization_msgs::Marker createMarkerMsg(const std::vector<geometry_msgs::Pose> &poses, float scale);
visualization_msgs::Marker createMarkerMsg(const std::vector<geometry_msgs::Point> &points, float scale, const std::vector<float> &&color);
mae_utils::PointArray createPointArrayMsg(const std::vector<geometry_msgs::Point> &points);
geometry_msgs::Pose poseFromVec(const std::vector<double> pos, const std::vector<double> rpy = {0, 0, 0});
geometry_msgs::Point pointFromPose(const geometry_msgs::Pose pose);
geometry_msgs::PoseStamped poseStampedFromVec(const std::vector<double> pos, const std::vector<double> rpy = {0, 0, 0});
float dist2D(const geometry_msgs::Point p1, const geometry_msgs::Point p2);
float dist2D(const geometry_msgs::Pose p1, const geometry_msgs::Pose p2);
float dist3D(const geometry_msgs::Point p1, const geometry_msgs::Point p2);
float dist3D(const geometry_msgs::Pose p1, const geometry_msgs::Pose p2);
geometry_msgs::Twist twistFromVec(const std::vector<double> linear = {0, 0, 0}, std::vector<double> angular = {0, 0, 0});
geometry_msgs::PoseStamped poseStampedFromPose(const geometry_msgs::Pose pose);
geometry_msgs::PoseStamped poseStampedFromPoint(const geometry_msgs::Point point);
template <typename T>
void log_many(std::vector<T> vec);

#endif // MAE_UTILS_H