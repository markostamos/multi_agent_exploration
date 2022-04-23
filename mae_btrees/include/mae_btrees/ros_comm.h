#ifndef ROS_COMM_H
#define ROS_COMM_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
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
    Pubs publishers;
};

void initRosComm(ros::NodeHandle &nh);

#endif // ROS_COMM_H