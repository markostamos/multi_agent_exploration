#ifndef ROS_COMM_H
#define ROS_COMM_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <mae_utils/PointArray.h>
struct Pubs
{
    ros::Publisher commandVel;
};

struct RosComm
{
    ros::NodeHandle *nh;
    std::string ns;

    // Agent current position
    geometry_msgs::Pose pose;

    // Frontier points
    std::vector<geometry_msgs::Point> frontier_pts;

    // Received plan points
    std::vector<geometry_msgs::Point> plan_pts;

    // Current battery percentage
    float battery_percentage;

    Pubs publishers;

    // Positions of other agents
    std::map<int, geometry_msgs::Point> drone_positions;

    // Task requested by user
    std::string requested_task;
};

void initRosComm(ros::NodeHandle &nh);

void waitForConnection();
#endif // ROS_COMM_H