#ifndef ROS_COMM_H
#define ROS_COMM_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
struct Pubs{
    ros::Publisher commandPose;
    ros::Publisher commandVel;
};

struct RosComm{
    ros::NodeHandle* nh;
    std::string ns;
    geometry_msgs::Pose pose;
    Pubs publishers;

}state;



void initRosComm(ros::NodeHandle &nh){
    state.nh = &nh;
    state.ns = nh.getNamespace().c_str();
    state.publishers.commandPose = nh.advertise<geometry_msgs::PoseStamped>(state.ns+"/command/pose",100);
    state.publishers.commandVel = nh.advertise<geometry_msgs::Twist>(state.ns+"/command/cmd_vel",100);
    
}

#endif // ROS_COMM_H