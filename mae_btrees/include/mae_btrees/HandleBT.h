#ifndef HANDLE_BT_H
#define HANDLE_BT_H
#include <ros/ros.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <mae_utils/PointArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
class HandleBT
{
public:
    BT::Tree tree_;

public:
    HandleBT(ros::NodeHandle &nh);

    void createTree(std::string path);

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_subscriber_;
    ros::Subscriber frontier_subscriber_;
    ros::Subscriber plan_subscriber_;
    ros::Subscriber task_subscriber_;
    ros::Publisher active_task_publisher_;
    ros::Subscriber lidar_readings_subscriber_;

    ros::Timer timer_;

private:
    void subPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void subFrontierCallback(const mae_utils::PointArray::ConstPtr &msg);
    void subDronePositionsCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void subPlanCallback(const mae_utils::PointArray::ConstPtr &msg);
    void subTaskCallback(const std_msgs::String::ConstPtr &msg);
    void pubActiveTaskCallback(const ros::TimerEvent &event);
    void subLidarReadingsCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
};

#endif // HANDLE_BT_H