#include <mae_btrees/HandleBT.h>
#include <mae_btrees/conditions.h>
extern RosComm state;

HandleBT::HandleBT(ros::NodeHandle &nh) : nh_(nh)
{

    initRosComm(nh_);

    pose_subscriber_ = nh_.subscribe(state.ns + "/ground_truth/odometry", 100, &HandleBT::subPoseCallback, this);
    frontier_subscriber_ = nh_.subscribe(state.ns + "/frontiers", 100, &HandleBT::subFrontierCallback, this);
    plan_subscriber_ = nh_.subscribe(state.ns + "/plan", 100, &HandleBT::subPlanCallback, this);
    drone_position_subscriber_ = nh_.subscribe(state.ns + "/comm/drone_positions", 100, &HandleBT::subDronePositionsCallback, this);
    activity_subscriber_ = nh_.subscribe(state.ns + "/task", 100, &HandleBT::subActivityCallback, this);
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
    factory.registerNodeType<SetNextTargetFromPlan>("SetNextTargetFromPlan");

    factory.registerNodeType<SetTask>("SetTask");
    factory.registerNodeType<ClearTask>("ClearTask");
    factory.registerNodeType<TestActivity>("TestActivity");
    factory.registerSimpleAction("Recharge", std::bind(Recharge));
    // REGISTER CONDITIONS
    factory.registerSimpleCondition("TaskAvailable", std::bind(TaskAvailable));
    factory.registerSimpleCondition("LowBattery", std::bind(LowBattery));

    factory.registerSimpleCondition("isFrontierListEmpty", std::bind(isFrontierListEmpty));
    factory.registerNodeType<TargetDiscovered>("TargetDiscovered");
    factory.registerNodeType<GreedyTargetDiscovered>("GreedyTargetDiscovered");
    factory.registerSimpleCondition("isNearOtherDrones", std::bind(isNearOtherDrones));
    factory.registerSimpleCondition("NewPlanArrived", std::bind(NewPlanArrived));
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
        tree_.blackboard_stack.front()->set<geometry_msgs::Pose>("Home", msg->pose.pose);
        ROS_WARN_STREAM("Home set to: " << msg->pose.pose);
        first_msg = false;
    }
    state.pose = msg->pose.pose;
};

void HandleBT::subFrontierCallback(const mae_utils::PointArray::ConstPtr &msg)
{
    state.frontier_pts = msg->points;
};

void HandleBT::subDronePositionsCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    state.drone_positions[atoi(msg->header.frame_id.c_str())] = msg->point;
}

void HandleBT::subPlanCallback(const mae_utils::PointArray::ConstPtr &msg)
{
    // get all poitns from msg except from first

    if (msg->points.size() > 1)
        state.plan_pts = std::vector<geometry_msgs::Point>(msg->points.begin() + 1, msg->points.end());
}

void HandleBT::subActivityCallback(const std_msgs::String::ConstPtr &msg)
{
    state.requested_task = msg->data;
}