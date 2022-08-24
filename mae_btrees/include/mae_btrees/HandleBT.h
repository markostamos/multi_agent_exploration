#ifndef HANDLE_BT_H
#define HANDLE_BT_H
#include <mae_btrees/actions.h>
#include <mae_btrees/conditions.h>
#include <mae_btrees/utils.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <mae_utils/PointArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
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
    ros::Subscriber drone_position_subscriber_;

private:
    void subPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void subFrontierCallback(const mae_utils::PointArray::ConstPtr &msg);
    void subDronePositionsCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
};

#endif // HANDLE_BT_H