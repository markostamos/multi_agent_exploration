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

    /**
     * @brief Load a Behavior Tree xml file and create the nodes
     *
     * @param path  Path to xml file
     */
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
    /**
     * @brief Subscriber callback for current pose of agent.
     *
     * @param msg   Odometry message
     */
    void subPoseCb(const nav_msgs::Odometry::ConstPtr &msg);

    /**
     * @brief Subscriber callback for list of frontiers points.
     *
     * @param msg PointArray message
     */
    void subFrontierCb(const mae_utils::PointArray::ConstPtr &msg);

    /**
     * @brief Subscriber callback for current plan of agent.
     *
     * @param msg PointArray message
     */
    void subPlanCb(const mae_utils::PointArray::ConstPtr &msg);

    /**
     * @brief Subscriber callback for task selection.
     *
     * @param msg String task name according to xml file.
     */
    void subTaskCb(const std_msgs::String::ConstPtr &msg);

    /**
     * @brief Publishes the active task.
     *
     * @param msg PointCloud2 message.
     */
    void pubActiveTaskCb(const ros::TimerEvent &event);

    /**
     * @brief Subscriber callback for agent's lidar readings.
     *
     * @param msg PointCloud2 message.
     */
    void subLidarReadingsCb(const sensor_msgs::PointCloud2::ConstPtr &msg);
};

#endif // HANDLE_BT_H