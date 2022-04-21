#ifndef HANDLE_BT_H
#define HANDLE_BT_H
#include <mae_btrees/actions.hpp>

extern RosComm state;
class InitBT
{

public:
    BT::Tree tree;
    InitBT(ros::NodeHandle &nh) : nh(nh)
    {

        initRosComm(nh);
        pose_subscriber = nh.subscribe(state.ns + "/ground_truth/pose", 100, &InitBT::subPositionCallback, this);
    }

    void createTree(std::string path);

    ros::NodeHandle nh;
    ros::Subscriber pose_subscriber;

    void subPositionCallback(const geometry_msgs::Pose::ConstPtr &msg);
};

void InitBT::createTree(std::string path)
{
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<GoTo>("GoTo");
    factory.registerNodeType<SetLocation>("SetLocation");
    factory.registerNodeType<TakeOff>("TakeOff");
    factory.registerNodeType<Land>("Land");

    tree = factory.createTreeFromFile(path);
}

/*
    SUBSCRIBER CALLBACKS
 */
void InitBT::subPositionCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    /* ree.blackboard_stack.back()->set<geometry_msgs::Pose>("pose", *msg); */
    state.pose = *msg;
};

#endif // HANDLE_BT_H