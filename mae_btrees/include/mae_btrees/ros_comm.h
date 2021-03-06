#ifndef ROS_COMM_H
#define ROS_COMM_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mae_utils/PointArray.h>
struct Pubs
{
    ros::Publisher commandPose;
    ros::Publisher commandVel;
};

struct RosComm
{
    ros::NodeHandle *nh;
    std::string ns;
    geometry_msgs::Pose pose;

    std::vector<geometry_msgs::Point> frontier_pts;
    Pubs publishers;

    bool cancel_goal_req;
    geometry_msgs::Pose next_target;
};

void initRosComm(ros::NodeHandle &nh);

void waitForConnection();
#endif // ROS_COMM_H