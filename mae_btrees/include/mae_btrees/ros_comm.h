#ifndef ROS_COMM_H
#define ROS_COMM_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <mae_utils/PointArray.h>

/**
 * @brief Struct that holds data from the ROS environment to be used in the Behavior Tree.
 *
 */
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

    // lidar sensor latest readings
    sensor_msgs::PointCloud2 lidar_readings;

    // Current battery percentage
    float battery_percentage;
    // Task requested by user
    std::string requested_task;
    bool task_arrived;
};

/**
 * @brief Initialize the RosComm struct variables.
 *
 * @param nh
 */
void initRosComm(ros::NodeHandle &nh);

#endif // ROS_COMM_H