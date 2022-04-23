#include <mae_frontier_generation/utils.h>
geometry_msgs::Pose poseFrom2DMapIndex(const int i, const int j, const nav_msgs::OccupancyGrid &map)
{
    geometry_msgs::Pose pose;
    pose.position.x = map.info.origin.position.x + i * map.info.resolution;
    pose.position.y = map.info.origin.position.y + j * map.info.resolution;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    return pose;
}

// point from map index
geometry_msgs::Point pointFrom2DMapIndex(const int i, const int j, const nav_msgs::OccupancyGrid &map)
{
    geometry_msgs::Point point;
    point.x = map.info.origin.position.x + i * map.info.resolution;
    point.y = map.info.origin.position.y + j * map.info.resolution;
    point.z = 0;
    return point;
}

// single marker msg of sphere lists from vector of poses
visualization_msgs::Marker createMarkerMsg(const std::vector<geometry_msgs::Pose> &poses)
{
    visualization_msgs::Marker msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.ns = "viz";
    msg.id = 0;
    msg.type = visualization_msgs::Marker::SPHERE_LIST;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = 0.2;
    msg.scale.y = 0.2;
    msg.scale.z = 0.2;
    msg.color.a = 1.0;
    msg.color.r = 1.0;
    msg.color.g = 0.0;
    msg.color.b = 1.0;
    for (int i = 0; i < poses.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = poses[i].position.x;
        p.y = poses[i].position.y;
        p.z = poses[i].position.z;
        p.z = 1;
        msg.points.push_back(p);
    }
    return msg;
}

// create marker msg from points
visualization_msgs::Marker createMarkerMsg(const std::vector<geometry_msgs::Point> &points)
{
    visualization_msgs::Marker msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.ns = "viz";
    msg.id = 0;
    msg.type = visualization_msgs::Marker::SPHERE_LIST;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = 0.2;
    msg.scale.y = 0.2;
    msg.scale.z = 0.2;
    msg.color.a = 1.0;
    msg.color.r = 1.0;
    msg.color.g = 0.0;
    msg.color.b = 1.0;
    for (int i = 0; i < points.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = points[i].x;
        p.y = points[i].y;
        p.z = points[i].z;
        p.z = 1;
        msg.points.push_back(p);
    }
    return msg;
}
