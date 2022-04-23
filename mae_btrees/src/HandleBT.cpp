#include <mae_btrees/HandleBT.h>

extern RosComm state;

HandleBT::HandleBT(ros::NodeHandle &nh) : nh_(nh)
{

    initRosComm(nh_);
    pose_subscriber_ = nh_.subscribe(state.ns + "/ground_truth/pose", 100, &HandleBT::subPositionCallback, this);
}

void HandleBT::createTree(std::string path)
{
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<GoTo>("GoTo");
    factory.registerNodeType<SetLocation>("SetLocation");
    factory.registerNodeType<TakeOff>("TakeOff");
    factory.registerNodeType<Land>("Land");

    tree_ = factory.createTreeFromFile(path);
}

/*
    SUBSCRIBER CALLBACKS
 */
void HandleBT::subPositionCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    /* ree.blackboard_stack.back()->set<geometry_msgs::Pose>("pose", *msg); */
    state.pose = *msg;
};