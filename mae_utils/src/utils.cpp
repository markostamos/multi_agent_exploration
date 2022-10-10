#include <mae_utils/utils.h>
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
visualization_msgs::Marker createMarkerMsg(const std::vector<geometry_msgs::Pose> &poses, float scale)
{
    visualization_msgs::Marker msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.ns = "viz";
    msg.id = 0;
    msg.type = visualization_msgs::Marker::SPHERE_LIST;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;
    msg.color.a = 1.0;
    msg.color.r = 1.0;
    msg.color.g = 0.0;
    msg.color.b = 1.0;
    for (int i = 0; i < poses.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = poses[i].position.x;
        p.y = poses[i].position.y;
        p.z = 0;
        msg.points.push_back(p);
    }
    return msg;
}

// create marker msg from points
visualization_msgs::Marker createMarkerMsg(const std::vector<geometry_msgs::Point> &points, float scale)
{
    visualization_msgs::Marker msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.ns = "viz";
    msg.id = 0;
    msg.type = visualization_msgs::Marker::SPHERE_LIST;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;
    msg.color.a = 1.0;
    msg.color.r = 1.0;
    msg.color.g = 0.0;
    msg.color.b = 1.0;
    for (int i = 0; i < points.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = points[i].x;
        p.y = points[i].y;
        p.z = points[i].z > 0.5 ? points[i].z : 0.5;

        msg.points.push_back(p);
    }
    return msg;
}

mae_utils::PointArray createPointArrayMsg(const std::vector<geometry_msgs::Point> &points)
{
    mae_utils::PointArray msg;
    msg.points = points;
    return msg;
}

geometry_msgs::Pose poseFromVec(std::vector<double> pos, std::vector<double> rpy)
{
    geometry_msgs::Pose msg;
    msg.position.x = pos[0];
    msg.position.y = pos[1];
    msg.position.z = pos[2];
    tf::Quaternion myQuaternion;
    myQuaternion.setRPY(rpy[0], rpy[1], rpy[2]);
    msg.orientation.x = myQuaternion.x();
    msg.orientation.y = myQuaternion.y();
    msg.orientation.z = myQuaternion.z();
    msg.orientation.w = myQuaternion.w();
    return msg;
}

geometry_msgs::PoseStamped poseStampedFromVec(std::vector<double> pos, std::vector<double> rpy)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.pose = poseFromVec(pos, rpy);
    return msg;
}

geometry_msgs::Twist twistFromVec(std::vector<double> linear, std::vector<double> angular)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear[0];
    msg.linear.y = linear[1];
    msg.linear.z = linear[2];
    msg.angular.x = angular[0];
    msg.angular.y = angular[1];
    msg.angular.z = angular[2];
    return msg;
}

float dist2D(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

float dist2D(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    return sqrt(pow(p1.position.x - p2.position.x, 2) + pow(p1.position.y - p2.position.y, 2));
}

float dist3D(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

float dist3D(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    return sqrt(pow(p1.position.x - p2.position.x, 2) + pow(p1.position.y - p2.position.y, 2) + pow(p1.position.z - p2.position.z, 2));
}

geometry_msgs::Point pointFromPose(geometry_msgs::Pose pose)
{
    geometry_msgs::Point p;
    p.x = pose.position.x;
    p.y = pose.position.y;
    p.z = pose.position.z;
    return p;
}

geometry_msgs::PoseStamped poseStampedFromPose(geometry_msgs::Pose pose)
{
    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = "world";
    p.pose = pose;
    return p;
}

geometry_msgs::PoseStamped poseStampedFromPoint(geometry_msgs::Point point)
{
    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = "world";
    p.pose.position = point;
    p.pose.orientation.w = 1;
    return p;
}
