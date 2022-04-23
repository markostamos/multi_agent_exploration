#ifndef HANDLE_BT_H
#define HANDLE_BT_H
#include <mae_btrees/actions.h>
#include <mae_btrees/utils.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
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

private:
    void subPositionCallback(const geometry_msgs::Pose::ConstPtr &msg);
};

#endif // HANDLE_BT_H