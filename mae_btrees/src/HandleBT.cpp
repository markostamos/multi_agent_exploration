#include <mae_btrees/HandleBT.h>
#include <mae_btrees/conditions.h>
extern RosComm state;

HandleBT::HandleBT(ros::NodeHandle &nh) : nh_(nh)
{

    initRosComm(nh_);

    pose_subscriber_ = nh_.subscribe(state.ns + "/ground_truth/odometry", 100, &HandleBT::subPoseCallback, this);
    frontier_subscriber_ = nh_.subscribe("/frontiers", 100, &HandleBT::subFrontierCallback, this);
    drone_position_subscriber_ = nh_.subscribe(state.ns + "/comm/drone_positions", 100, &HandleBT::subDronePositionsCallback, this);
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
    factory.registerSimpleCondition("isNearOtherDrones", std::bind(isNearOtherDrones));
    tree_ = factory.createTreeFromFile(path);
}

/*
    SUBSCRIBER CALLBACKS
 */
void HandleBT::subPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    static bool first_msg = true;
    if (first_msg)
    {
        tree_.blackboard_stack.back()->set<geometry_msgs::Pose>("home", msg->pose.pose);
        first_msg = false;
    }
    state.pose = msg->pose.pose;
    tree_.blackboard_stack.back()->set<geometry_msgs::Pose>("stay", msg->pose.pose);
};

void HandleBT::subFrontierCallback(const mae_utils::PointArray::ConstPtr &msg)
{
    state.frontier_pts = msg->points;
};

void HandleBT::subDronePositionsCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    state.drone_positions[atoi(msg->header.frame_id.c_str())] = msg->point;
}