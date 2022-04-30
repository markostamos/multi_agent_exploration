#include <mae_btrees/HandleBT.h>
#include <mae_btrees/conditions.h>
extern RosComm state;

HandleBT::HandleBT(ros::NodeHandle &nh) : nh_(nh)
{

    initRosComm(nh_);

    pose_subscriber_ = nh_.subscribe(state.ns + "/ground_truth/pose", 100, &HandleBT::subPositionCallback, this);
    frontier_subscriber_ = nh_.subscribe(state.ns + "/frontiers", 100, &HandleBT::subFrontierCallback, this);

    waitForConnection();
}

void HandleBT::createTree(std::string path)
{
    BT::BehaviorTreeFactory factory;

    // REGISTER ACTIONS
    factory.registerNodeType<GoTo>("GoTo");
    factory.registerNodeType<SetLocation>("SetLocation");
    factory.registerNodeType<TakeOff>("TakeOff");
    factory.registerNodeType<Land>("Land");
    factory.registerNodeType<SetNextTargetGreedy>("SetNextTargetGreedy");
    factory.registerSimpleAction("CancelGoal", std::bind(CancelGoal));
    // REGISTER CONDITIONS
    factory.registerSimpleCondition("isFrontierListEmpty", std::bind(isFrontierListEmpty));
    factory.registerSimpleCondition("TargetDiscovered", std::bind(TargetDiscovered));
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

void HandleBT::subFrontierCallback(const mae_utils::PointArray::ConstPtr &msg)
{
    state.frontier_pts = msg->points;
};
