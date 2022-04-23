#ifndef UTILS_H
#define UTILS_H
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

geometry_msgs::Pose poseFrom2DMapIndex(const int i, const int j, const nav_msgs::OccupancyGrid &map);
geometry_msgs::Point pointFrom2DMapIndex(const int i, const int j, const nav_msgs::OccupancyGrid &map);
visualization_msgs::Marker createMarkerMsg(const std::vector<geometry_msgs::Pose> &poses);
visualization_msgs::Marker createMarkerMsg(const std::vector<geometry_msgs::Point> &points);

#endif // UTILS_H
